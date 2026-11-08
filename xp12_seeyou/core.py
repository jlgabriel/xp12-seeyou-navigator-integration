"""Flight data model, vario handling and NMEA generation (Condor 3 format)."""

import datetime
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple

# ── Constants ────────────────────────────────────────────────────────

BAUD_RATES = (4800, 9600, 19200, 38400)
DEFAULT_BAUDRATE = 4800           # NMEA standard, same as Condor 3
DEFAULT_COM_PORT = "COM7"
DEFAULT_UPDATE_RATE = 1.0         # Hz, same as Condor 3

BARO_MODES = ("msl", "pressure")  # LXWP0 altitude: true MSL (Condor) or pressure alt (LX)
TIME_SOURCES = ("sim", "system")  # NMEA UTC time: X-Plane zulu time (Condor) or PC clock
VARIO_MODES = ("auto", "te", "raw")

MS_TO_KPH = 3.6
MS_TO_KTS = 1.9438445
KTS_TO_MS = 0.5144444
FT_TO_M = 0.3048
FPM_TO_MS = 0.00508

# Datarefs the bridge reads: (key, path, type). Type is used by the plugin
# ('d' double, 'f' float, 'i' int); over RREF everything arrives as float32.
# All of them exist in X-Plane 12 (checked against DataRefs.txt of 12.4).
DATAREFS: List[Tuple[str, str, str]] = [
    ("lat",             "sim/flightmodel/position/latitude",               "d"),  # deg
    ("lon",             "sim/flightmodel/position/longitude",              "d"),  # deg
    ("elev_m",          "sim/flightmodel/position/elevation",              "d"),  # m MSL
    ("pressure_alt_ft", "sim/flightmodel2/position/pressure_altitude",      "d"),  # ft (QNE)
    ("agl_m",           "sim/flightmodel/position/y_agl",                  "f"),  # m
    ("gs_ms",           "sim/flightmodel/position/groundspeed",            "f"),  # m/s
    ("vx",              "sim/flightmodel/position/local_vx",               "f"),  # m/s, +x = east
    ("vz",              "sim/flightmodel/position/local_vz",               "f"),  # m/s, +z = south
    ("true_psi",        "sim/flightmodel/position/true_psi",               "f"),  # deg true
    ("mag_var",         "sim/flightmodel/position/magnetic_variation",     "f"),  # deg
    ("ias_kt",          "sim/flightmodel/position/indicated_airspeed",     "f"),  # kias
    ("tas_ms",          "sim/flightmodel/position/true_airspeed",          "f"),  # m/s
    ("vs_ms",           "sim/flightmodel/position/vh_ind",                 "f"),  # m/s
    ("te_fpm",          "sim/cockpit2/gauges/indicators/total_energy_fpm", "f"),  # ft/min
    ("wind_dir",        "sim/weather/aircraft/wind_now_direction_degt",    "f"),  # deg true, FROM
    ("wind_ms",         "sim/weather/aircraft/wind_now_speed_msc",         "f"),  # m/s
    ("zulu_sec",        "sim/time/zulu_time_sec",                          "f"),  # s since midnight
    ("paused",          "sim/time/paused",                                 "i"),  # 0/1
]
KEY_INDEX: Dict[str, int] = {key: i for i, (key, _, _) in enumerate(DATAREFS)}

# ── Data classes ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class FlightData:
    """Snapshot of the aircraft state, already converted to SI units."""
    timestamp: float = 0.0                      # PC time of the data
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_msl_m: float = 0.0
    pressure_altitude_m: Optional[float] = None
    agl_m: Optional[float] = None
    ground_speed_ms: float = 0.0
    track_deg: float = 0.0
    heading_deg: float = 0.0
    magnetic_variation: Optional[float] = None
    ias_ms: Optional[float] = None
    tas_ms: Optional[float] = None
    vario_ms: Optional[float] = None            # selected source, averaged over 1 s
    vario_avg_ms: Optional[float] = None        # selected source, averaged over 30 s
    vario_source: str = "---"
    wind_dir_deg: Optional[float] = None        # direction the wind blows FROM
    wind_speed_ms: Optional[float] = None
    zulu_sec: Optional[float] = None
    paused: bool = False

    @property
    def has_position(self) -> bool:
        return self.timestamp > 0 and not (self.latitude == 0.0 and self.longitude == 0.0)

# ── Vario ────────────────────────────────────────────────────────────

class RollingMean:
    """Mean of the samples received in the last `window` seconds."""

    def __init__(self, window: float):
        self.window = window
        self._samples: deque = deque()

    def add(self, t: float, value: float) -> None:
        self._samples.append((t, value))
        cutoff = t - self.window
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def mean(self) -> Optional[float]:
        if not self._samples:
            return None
        return sum(v for _, v in self._samples) / len(self._samples)


class VarioSelector:
    """
    Chooses between the total-energy vario and the raw vertical speed.

    'te' and 'raw' are fixed. 'auto' prefers total energy, but falls back to raw
    if the TE dataref stays at exactly 0 while the aircraft is clearly climbing
    or sinking (an aircraft whose TE gauge is not simulated).
    """
    DEAD_TE_SECONDS = 3.0
    RAW_THRESHOLD_MS = 0.3

    def __init__(self, mode: str = "auto"):
        if mode not in VARIO_MODES:
            raise ValueError(f"Unknown vario mode: {mode}")
        self.mode = mode
        self.te_dead = False
        self._te_zero_since: Optional[float] = None

    def observe(self, t: float, te_ms: float, raw_ms: float) -> None:
        if te_ms == 0.0 and abs(raw_ms) > self.RAW_THRESHOLD_MS:
            if self._te_zero_since is None:
                self._te_zero_since = t
            elif t - self._te_zero_since >= self.DEAD_TE_SECONDS:
                self.te_dead = True
        elif te_ms != 0.0:
            self._te_zero_since = None
            self.te_dead = False

    @property
    def source(self) -> str:
        if self.mode == "raw" or (self.mode == "auto" and self.te_dead):
            return "raw"
        return "te"


class VarioTracker:
    """TE and raw vario samples, 1 s and 30 s averages of the selected source."""

    def __init__(self, mode: str = "auto"):
        self.selector = VarioSelector(mode)
        self._short = {"te": RollingMean(1.0), "raw": RollingMean(1.0)}
        self._long = {"te": RollingMean(30.0), "raw": RollingMean(30.0)}

    @property
    def mode(self) -> str:
        return self.selector.mode

    @mode.setter
    def mode(self, value: str) -> None:
        if value not in VARIO_MODES:
            raise ValueError(f"Unknown vario mode: {value}")
        self.selector.mode = value

    def add(self, t: float, te_ms: float, raw_ms: float) -> None:
        self.selector.observe(t, te_ms, raw_ms)
        for source, value in (("te", te_ms), ("raw", raw_ms)):
            self._short[source].add(t, value)
            self._long[source].add(t, value)

    def add_from_values(self, t: float, values: Mapping[str, float]) -> None:
        """Feed one sample from a {dataref key: value} map; skipped while paused.

        A missing TE dataref counts as a dead TE gauge (0), so 'auto' falls back to raw.
        """
        te_fpm, vs_ms = values.get("te_fpm"), values.get("vs_ms")
        if vs_ms is None or (values.get("paused") or 0) >= 0.5:
            return
        self.add(t, (te_fpm or 0.0) * FPM_TO_MS, vs_ms)

    def current(self) -> Tuple[Optional[float], Optional[float], str]:
        """(1 s average, 30 s average, 'te' or 'raw')."""
        source = self.selector.source
        return self._short[source].mean(), self._long[source].mean(), source

# ── Conversions ──────────────────────────────────────────────────────

def track_from_velocity(vx: float, vz: float) -> float:
    """Ground track in degrees true from X-Plane local OGL velocity (+x east, +z south)."""
    return math.degrees(math.atan2(vx, -vz)) % 360.0


def ias_to_tas(ias_ms: float, altitude_m: float) -> float:
    """IAS to TAS with the ISA troposphere model (only used as a fallback)."""
    h = max(0.0, min(altitude_m, 11000.0))
    t = 288.15 - 0.0065 * h
    density_ratio = (t / 288.15) ** (9.80665 / (287.058 * 0.0065) - 1.0)
    return ias_ms / math.sqrt(density_ratio)


def build_flight_data(values: Mapping[str, float], timestamp: float,
                      vario: VarioTracker) -> FlightData:
    """Turn a {dataref key: raw value} map into a FlightData snapshot."""
    get = values.get
    if timestamp <= 0 or get("lat") is None or get("lon") is None:
        return FlightData()

    gs = get("gs_ms") or 0.0
    heading = (get("true_psi") or 0.0) % 360.0
    vx, vz = get("vx"), get("vz")
    if gs > 1.0 and vx is not None and vz is not None:
        track = track_from_velocity(vx, vz)
    else:
        track = heading  # on the ground / hovering: velocity direction is noise
    elev = get("elev_m") or 0.0
    pressure_alt_ft = get("pressure_alt_ft")
    ias_kt = get("ias_kt")
    tas = get("tas_ms")
    if tas is None and ias_kt is not None:
        tas = ias_to_tas(ias_kt * KTS_TO_MS, elev)
    vario_short, vario_long, source = vario.current()

    return FlightData(
        timestamp=timestamp,
        latitude=get("lat"),
        longitude=get("lon"),
        altitude_msl_m=elev,
        pressure_altitude_m=pressure_alt_ft * FT_TO_M if pressure_alt_ft is not None else None,
        agl_m=get("agl_m"),
        ground_speed_ms=gs,
        track_deg=track,
        heading_deg=heading,
        magnetic_variation=get("mag_var"),
        ias_ms=ias_kt * KTS_TO_MS if ias_kt is not None else None,
        tas_ms=tas,
        vario_ms=vario_short,
        vario_avg_ms=vario_long,
        vario_source=source.upper() if vario_short is not None else "---",
        wind_dir_deg=get("wind_dir"),
        wind_speed_ms=get("wind_ms"),
        zulu_sec=get("zulu_sec"),
        paused=(get("paused") or 0) >= 0.5,
    )

# ── NMEA ─────────────────────────────────────────────────────────────

class NMEAConverter:
    """Builds the three sentences Condor 3 sends to the Naviter dongle."""

    def __init__(self, baro_mode: str = "msl", time_source: str = "sim"):
        if baro_mode not in BARO_MODES:
            raise ValueError(f"Unknown baro mode: {baro_mode}")
        if time_source not in TIME_SOURCES:
            raise ValueError(f"Unknown time source: {time_source}")
        self.baro_mode = baro_mode
        self.time_source = time_source

    def create_sentences(self, fd: FlightData) -> List[str]:
        """Order matches Condor 3: GPGGA, GPRMC, LXWP0."""
        if not fd.has_position:
            return []
        time_str = self._time_string(fd)
        return [self._gga(fd, time_str), self._rmc(fd, time_str), self._lxwp0(fd)]

    def _time_string(self, fd: FlightData) -> str:
        """hhmmss.sss — Condor 3 sends simulator time, so 'sim' is the default."""
        if self.time_source == "sim" and fd.zulu_sec is not None:
            total_ms = int(round(fd.zulu_sec * 1000)) % 86_400_000
        else:
            now = datetime.datetime.now(datetime.timezone.utc)
            total_ms = ((now.hour * 60 + now.minute) * 60 + now.second) * 1000 \
                + now.microsecond // 1000
        seconds, ms = divmod(total_ms, 1000)
        h, rem = divmod(seconds, 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}{m:02d}{s:02d}.{ms:03d}"

    def _gga(self, fd: FlightData, time_str: str) -> str:
        lat, ns = self.latitude_to_nmea(fd.latitude)
        lon, ew = self.longitude_to_nmea(fd.longitude)
        # Condor 3: 12 satellites, HDOP 10, empty geoid/DGPS fields, trailing 0000
        return self.wrap(",".join([
            "GPGGA", time_str, lat, ns, lon, ew,
            "1", "12", "10", f"{fd.altitude_msl_m:.1f}", "M",
            "", "", "", "", "0000",
        ]))

    def _rmc(self, fd: FlightData, time_str: str) -> str:
        lat, ns = self.latitude_to_nmea(fd.latitude)
        lon, ew = self.longitude_to_nmea(fd.longitude)
        # Condor 3: empty date, magnetic variation and mode fields
        return self.wrap(",".join([
            "GPRMC", time_str, "A", lat, ns, lon, ew,
            f"{fd.ground_speed_ms * MS_TO_KTS:.2f}", f"{fd.track_deg:.2f}",
            "", "", "", "",
        ]))

    def _lxwp0(self, fd: FlightData) -> str:
        """$LXWP0,Y,<TAS kph>,<baro alt m>,<vario m/s>,,,,,,<heading>,<wind dir>,<wind kph>"""
        tas = fd.tas_ms if fd.tas_ms is not None else fd.ground_speed_ms
        if self.baro_mode == "pressure" and fd.pressure_altitude_m is not None:
            baro_alt = fd.pressure_altitude_m
        else:
            baro_alt = fd.altitude_msl_m
        vario = fd.vario_ms if fd.vario_ms is not None else 0.0
        wind_dir = fd.wind_dir_deg if fd.wind_dir_deg is not None else 0.0
        wind_kph = (fd.wind_speed_ms or 0.0) * MS_TO_KPH
        return self.wrap(",".join([
            "LXWP0", "Y", f"{tas * MS_TO_KPH:.1f}", f"{baro_alt:.1f}", f"{vario:.2f}",
            "", "", "", "", "",
            f"{self.whole_degrees(fd.heading_deg)}",
            f"{self.whole_degrees(wind_dir)}",
            f"{wind_kph:.1f}",
        ]))

    @staticmethod
    def whole_degrees(angle: float) -> int:
        return int(round(angle)) % 360

    @staticmethod
    def _to_nmea(value: float, deg_width: int) -> str:
        # Round to 1/10000 minute first so 59.99999' never prints as "60.0000"
        total = int(round(abs(value) * 60 * 10000))
        degrees, rest = divmod(total, 60 * 10000)
        return f"{degrees:0{deg_width}d}{rest / 10000:07.4f}"

    @classmethod
    def latitude_to_nmea(cls, latitude: float) -> Tuple[str, str]:
        return cls._to_nmea(latitude, 2), ("S" if latitude < 0 else "N")

    @classmethod
    def longitude_to_nmea(cls, longitude: float) -> Tuple[str, str]:
        return cls._to_nmea(longitude, 3), ("W" if longitude < 0 else "E")

    @staticmethod
    def checksum(body: str) -> str:
        value = 0
        for char in body:
            value ^= ord(char)
        return f"{value:02X}"

    @classmethod
    def wrap(cls, body: str) -> str:
        return f"${body}*{cls.checksum(body)}"
