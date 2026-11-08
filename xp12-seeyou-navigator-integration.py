#!/usr/bin/env python3
# Copyright (c) 2026 Juan Luis Gabriel
# X-Plane 12 to SeeYou Navigator Bridge
# This software is released under the MIT License.

"""
Bridge between X-Plane 12 and SeeYou Navigator via serial COM port.

This program:
1. Subscribes to X-Plane datarefs over UDP (RREF protocol, port 49000).
   No plugin and no X-Plane configuration are needed.
2. Converts the data to NMEA sentences (GPGGA, GPRMC, LXWP0).
3. Sends the sentences through a serial port to the Naviter dongle,
   which forwards them to SeeYou Navigator.

The NMEA output matches what Condor 3 sends to the Naviter dongle,
so SeeYou Navigator treats X-Plane like Condor.
"""

import argparse
import datetime
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

VERSION = "1.0.0"

# ── Configuration ────────────────────────────────────────────────────

DEFAULT_XP_HOST = "127.0.0.1"     # X-Plane machine (localhost or LAN IP)
DEFAULT_XP_PORT = 49000           # X-Plane UDP receive port
DEFAULT_RREF_FREQ = 10            # Dataref updates per second requested from X-Plane
DEFAULT_COM_PORT = "COM7"         # Default serial port (Naviter dongle)
DEFAULT_BAUDRATE = 4800           # NMEA standard baud rate (same as Condor 3)
DEFAULT_UPDATE_RATE = 1.0         # NMEA output rate in Hz (same as Condor 3)
DEFAULT_DEBUG_LEVEL = 1           # 0=minimal, 1=normal, 2=verbose
BAUD_RATES = (4800, 9600, 19200, 38400)

BARO_MODES = ("msl", "pressure")  # LXWP0 altitude: true MSL (Condor) or pressure alt (LX)
TIME_SOURCES = ("sim", "system")  # NMEA UTC time: X-Plane zulu time (Condor) or PC clock
VARIO_MODES = ("auto", "te", "raw")

# Naviter dongle = Silicon Labs CP210x USB to UART bridge
NAVITER_DONGLE_VID = 0x10C4
NAVITER_DONGLE_PID = 0xEA60

MS_TO_KPH = 3.6
MS_TO_KTS = 1.9438445
KTS_TO_MS = 0.5144444
FT_TO_M = 0.3048
FPM_TO_MS = 0.00508

SETTINGS_DIR_NAME = "xp12-seeyou-bridge"

# Datarefs requested from X-Plane. The list index is the RREF index.
# All of them exist in X-Plane 12 (checked against DataRefs.txt of 12.4).
DATAREFS: List[Tuple[str, str]] = [
    ("lat",             "sim/flightmodel/position/latitude"),                 # deg
    ("lon",             "sim/flightmodel/position/longitude"),                # deg
    ("elev_m",          "sim/flightmodel/position/elevation"),                # m MSL
    ("pressure_alt_ft", "sim/flightmodel2/position/pressure_altitude"),       # ft (QNE)
    ("agl_m",           "sim/flightmodel/position/y_agl"),                    # m
    ("gs_ms",           "sim/flightmodel/position/groundspeed"),              # m/s
    ("vx",              "sim/flightmodel/position/local_vx"),                 # m/s, OGL +x = east
    ("vz",              "sim/flightmodel/position/local_vz"),                 # m/s, OGL +z = south
    ("true_psi",        "sim/flightmodel/position/true_psi"),                 # deg true
    ("mag_var",         "sim/flightmodel/position/magnetic_variation"),       # deg
    ("ias_kt",          "sim/flightmodel/position/indicated_airspeed"),       # kias
    ("tas_ms",          "sim/flightmodel/position/true_airspeed"),            # m/s
    ("vs_ms",           "sim/flightmodel/position/vh_ind"),                   # m/s
    ("te_fpm",          "sim/cockpit2/gauges/indicators/total_energy_fpm"),   # ft/min
    ("wind_dir",        "sim/weather/aircraft/wind_now_direction_degt"),      # deg true, FROM
    ("wind_ms",         "sim/weather/aircraft/wind_now_speed_msc"),           # m/s
    ("zulu_sec",        "sim/time/zulu_time_sec"),                            # s since midnight
    ("paused",          "sim/time/paused"),                                   # 0/1
]
KEY_INDEX: Dict[str, int] = {key: i for i, (key, _) in enumerate(DATAREFS)}

# ── Data classes ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class FlightData:
    """Snapshot of the aircraft state, already converted to SI units."""
    timestamp: float = 0.0                      # PC time of the last X-Plane packet
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

# ── Helpers ──────────────────────────────────────────────────────────

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

    def clear(self) -> None:
        self._samples.clear()


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


def track_from_velocity(vx: float, vz: float) -> float:
    """Ground track in degrees true from X-Plane local OGL velocity (+x east, +z south)."""
    return math.degrees(math.atan2(vx, -vz)) % 360.0


def ias_to_tas(ias_ms: float, altitude_m: float) -> float:
    """IAS to TAS with the ISA troposphere model (only used as a fallback)."""
    h = max(0.0, min(altitude_m, 11000.0))
    t = 288.15 - 0.0065 * h
    density_ratio = (t / 288.15) ** (9.80665 / (287.058 * 0.0065) - 1.0)
    return ias_ms / math.sqrt(density_ratio)


def settings_path() -> str:
    base = os.environ.get("APPDATA") or os.path.expanduser("~")
    return os.path.join(base, SETTINGS_DIR_NAME, "settings.json")


def load_settings() -> dict:
    try:
        with open(settings_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_settings(settings: dict) -> None:
    path = settings_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=2)
    except OSError as e:
        print(f"Could not save settings to {path}: {e}")

# ── X-Plane receiver (UDP RREF) ──────────────────────────────────────

class XPlaneReceiver:
    """
    Subscribes to X-Plane datarefs with the RREF UDP protocol.

    X-Plane answers to the address and port the request came from, so a single
    socket on an ephemeral port is enough. Subscriptions are re-sent while no data
    arrives, which covers X-Plane being started (or restarted) after the bridge.
    """
    RESUBSCRIBE_INTERVAL = 3.0
    STALE_TIMEOUT = 3.0

    def __init__(self, host: str = DEFAULT_XP_HOST, port: int = DEFAULT_XP_PORT,
                 freq: int = DEFAULT_RREF_FREQ, vario_mode: str = "auto",
                 debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.host = host
        self.port = port
        self.freq = freq
        self.debug_level = debug_level
        self.vario_selector = VarioSelector(vario_mode)
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._values: Dict[int, float] = {}
        self._vario = {"te": RollingMean(1.0), "raw": RollingMean(1.0)}
        self._vario_avg = {"te": RollingMean(30.0), "raw": RollingMean(30.0)}
        self.running = False
        self.last_receive_time = 0.0
        self.packets_received = 0

    # Protocol ----------------------------------------------------------

    @staticmethod
    def build_rref_request(freq: int, index: int, path: str) -> bytes:
        """RREF request: 'RREF\\0' + int32 freq + int32 index + char[400] path (413 bytes)."""
        return b"RREF\x00" + struct.pack("<ii400s", freq, index, path.encode("ascii"))

    @staticmethod
    def parse_rref_packet(data: bytes) -> List[Tuple[int, float]]:
        """RREF answer: 'RREF' + 1 byte + N x (int32 index, float32 value)."""
        if len(data) < 5 or data[:4] != b"RREF":
            return []
        payload = data[5:]
        return [struct.unpack_from("<if", payload, i * 8) for i in range(len(payload) // 8)]

    # Lifecycle ---------------------------------------------------------

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Windows reports ICMP "port unreachable" (X-Plane not running) as an error
        # on the next recvfrom(). Disable that so the loop keeps waiting quietly.
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            try:
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        sock.bind(("", 0))
        sock.settimeout(0.5)
        self._sock = sock
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"Requesting {len(DATAREFS)} datarefs from X-Plane at "
              f"{self.host}:{self.port} ({self.freq} Hz)")

    def stop(self):
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(1.5)
        if self._sock:
            self._subscribe(0)  # unsubscribe so X-Plane stops sending
            self._sock.close()
            self._sock = None

    def _subscribe(self, freq: int):
        for i, (_, path) in enumerate(DATAREFS):
            try:
                self._sock.sendto(self.build_rref_request(freq, i, path), (self.host, self.port))
            except OSError as e:
                if self.debug_level >= 2:
                    print(f"RREF request failed: {e}")
                return

    def _loop(self):
        last_subscribe = 0.0
        while self.running:
            now = time.time()
            if (now - self.last_receive_time > self.STALE_TIMEOUT
                    and now - last_subscribe >= self.RESUBSCRIBE_INTERVAL):
                if self.debug_level >= 2:
                    print("Subscribing to X-Plane datarefs...")
                self._subscribe(self.freq)
                last_subscribe = now
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.1)
                continue
            pairs = self.parse_rref_packet(data)
            if pairs:
                self._ingest(pairs, time.time())

    def _ingest(self, pairs: List[Tuple[int, float]], now: float):
        with self._lock:
            for index, value in pairs:
                if 0 <= index < len(DATAREFS):
                    self._values[index] = value
            self.last_receive_time = now
            self.packets_received += 1

            te_fpm = self._values.get(KEY_INDEX["te_fpm"])
            vs_ms = self._values.get(KEY_INDEX["vs_ms"])
            paused = self._values.get(KEY_INDEX["paused"], 0.0) >= 0.5
            if te_fpm is None or vs_ms is None or paused:
                return
            te_ms = te_fpm * FPM_TO_MS
            self.vario_selector.observe(now, te_ms, vs_ms)
            for source, value in (("te", te_ms), ("raw", vs_ms)):
                self._vario[source].add(now, value)
                self._vario_avg[source].add(now, value)

    # Data access -------------------------------------------------------

    def is_connected(self, timeout: float = STALE_TIMEOUT) -> bool:
        return (time.time() - self.last_receive_time) < timeout

    def snapshot(self) -> FlightData:
        with self._lock:
            values = dict(self._values)
            timestamp = self.last_receive_time
            source = self.vario_selector.source
            vario = self._vario[source].mean()
            vario_avg = self._vario_avg[source].mean()

        def get(key: str) -> Optional[float]:
            return values.get(KEY_INDEX[key])

        if timestamp == 0 or get("lat") is None or get("lon") is None:
            return FlightData()

        gs = get("gs_ms") or 0.0
        heading = (get("true_psi") or 0.0) % 360.0
        vx, vz = get("vx"), get("vz")
        if gs > 1.0 and vx is not None and vz is not None:
            track = track_from_velocity(vx, vz)
        else:
            track = heading  # on the ground / hovering: velocity direction is noise
        pressure_alt_ft = get("pressure_alt_ft")
        ias_kt = get("ias_kt")
        tas = get("tas_ms")
        elev = get("elev_m") or 0.0
        if tas is None and ias_kt is not None:
            tas = ias_to_tas(ias_kt * KTS_TO_MS, elev)

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
            vario_ms=vario,
            vario_avg_ms=vario_avg,
            vario_source=source.upper() if vario is not None else "---",
            wind_dir_deg=get("wind_dir"),
            wind_speed_ms=get("wind_ms"),
            zulu_sec=get("zulu_sec"),
            paused=(get("paused") or 0.0) >= 0.5,
        )

# ── NMEA Converter ────────────────────────────────────────────────────

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
        return [
            self._gga(fd, time_str),
            self._rmc(fd, time_str),
            self._lxwp0(fd),
        ]

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

# ── Output writers ───────────────────────────────────────────────────

class SerialWriter:
    """Writes NMEA sentences to the COM port of the Naviter dongle."""

    RECONNECT_INTERVAL = 3.0

    def __init__(self, port: str = DEFAULT_COM_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.port = port
        self.baudrate = baudrate
        self.debug_level = debug_level
        self._serial = None
        self._lock = threading.Lock()

    def start(self):
        self._connect()

    def _connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1,
            )
            print(f"Serial port {self.port} opened at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            if self.debug_level >= 1:
                print(f"Could not open {self.port}: {e}")
            self._serial = None
            return False

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send_batch(self, sentences: List[str]) -> bool:
        """Write all sentences of one cycle in a single call, each ended with CR+LF."""
        if not self.is_connected() or not sentences:
            return False
        payload = "".join(s + "\r\n" for s in sentences).encode("ascii")
        with self._lock:
            try:
                self._serial.write(payload)
                return True
            except serial.SerialException as e:
                if self.debug_level >= 1:
                    print(f"Serial write error: {e}")
                self._close()
                return False

    def reconnect(self):
        with self._lock:
            self._close()
        self._connect()

    def _close(self):
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def stop(self):
        with self._lock:
            self._close()

    @staticmethod
    def list_ports_detailed() -> List[Tuple[str, str, Optional[int], Optional[int]]]:
        """(device, description, vid, pid) for each COM port, sorted by name."""
        if not HAS_PYSERIAL:
            return []
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        return [(p.device, p.description, p.vid, p.pid) for p in ports]

    @classmethod
    def find_naviter_dongle(cls) -> Optional[str]:
        """COM port of the first Silicon Labs CP210x device, which is what the dongle uses."""
        for device, description, vid, pid in cls.list_ports_detailed():
            if (vid == NAVITER_DONGLE_VID and pid == NAVITER_DONGLE_PID) or "CP210" in description:
                return device
        return None


class ConsoleWriter:
    """Prints the sentences instead of writing to a COM port (--dry-run)."""

    RECONNECT_INTERVAL = 3.0

    def __init__(self):
        self.port = "console"
        self.baudrate = 0

    def start(self):
        print("Dry run: NMEA sentences are printed, no COM port is opened.")

    def is_connected(self) -> bool:
        return True

    def send_batch(self, sentences: List[str]) -> bool:
        for s in sentences:
            print(s)
        return bool(sentences)

    def reconnect(self):
        pass

    def stop(self):
        pass

# ── Orchestrator ──────────────────────────────────────────────────────

class XPlaneToSeeYou:
    """Glues the X-Plane receiver, the NMEA converter and the output writer together."""

    # Rough size of one GPGGA + GPRMC + LXWP0 cycle, used to check the baud rate
    BYTES_PER_CYCLE = 210

    def __init__(self, xp_host=DEFAULT_XP_HOST, xp_port=DEFAULT_XP_PORT,
                 com_port=DEFAULT_COM_PORT, baudrate=DEFAULT_BAUDRATE,
                 update_rate=DEFAULT_UPDATE_RATE, rref_freq=DEFAULT_RREF_FREQ,
                 baro_mode="msl", time_source="sim", vario_mode="auto",
                 debug_level=DEFAULT_DEBUG_LEVEL, dry_run=False, writer=None):
        self.xp_host = xp_host
        self.xp_port = xp_port
        self.com_port = com_port
        self.baudrate = baudrate
        self.update_rate = update_rate if update_rate > 0 else DEFAULT_UPDATE_RATE
        self.rref_freq = rref_freq
        self.vario_mode = vario_mode
        self.debug_level = debug_level
        self.dry_run = dry_run
        self._injected_writer = writer

        self.converter = NMEAConverter(baro_mode, time_source)
        self.receiver = self._make_receiver()
        self.writer = self._make_writer()

        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.start_time = 0.0
        self.sentences_count = 0
        self.last_sentences: List[str] = []

    def _make_receiver(self) -> XPlaneReceiver:
        return XPlaneReceiver(self.xp_host, self.xp_port, self.rref_freq,
                              self.vario_mode, self.debug_level)

    def _make_writer(self):
        if self._injected_writer is not None:
            return self._injected_writer
        if self.dry_run:
            return ConsoleWriter()
        return SerialWriter(self.com_port, self.baudrate, self.debug_level)

    def start_services(self):
        """Start (or restart, picking up changed settings) all background work."""
        print(f"X-Plane 12 to SeeYou Navigator bridge v{VERSION}")
        print(f"X-Plane: {self.xp_host}:{self.xp_port} | Output: "
              f"{'console' if self.dry_run else f'{self.com_port} @ {self.baudrate} baud'} | "
              f"{self.update_rate:g} Hz | baro alt: {self.converter.baro_mode} | "
              f"time: {self.converter.time_source} | vario: {self.vario_mode}")
        if not self.dry_run and self._injected_writer is None:
            needed = self.BYTES_PER_CYCLE * 10 * self.update_rate
            if needed > 0.9 * self.baudrate:
                print(f"WARNING: {self.update_rate:g} Hz needs ~{needed:.0f} baud; "
                      f"{self.baudrate} baud will fall behind. Lower the rate or raise the baud.")

        self.receiver = self._make_receiver()
        self.writer = self._make_writer()
        self.receiver.start()
        self.writer.start()

        self.sentences_count = 0
        self.start_time = time.time()
        self.running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()

    def start(self):
        """Terminal mode: start and block until Ctrl+C."""
        self.start_services()
        print("Press Ctrl+C to stop.\n")
        try:
            while self.running:
                time.sleep(0.5)
                if not self.receiver.is_connected() and not self.dry_run:
                    print("Waiting for data from X-Plane...", end="\r")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nStopping bridge...")
        self.stop()

    def stop(self):
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(1.5)
        self.receiver.stop()
        self.writer.stop()
        print("Bridge stopped.")

    def _process_loop(self):
        interval = 1.0 / self.update_rate
        next_cycle = time.time()
        last_stats = time.time()
        last_reconnect = 0.0

        while self.running:
            now = time.time()

            if not self.writer.is_connected() and now - last_reconnect >= self.writer.RECONNECT_INTERVAL:
                last_reconnect = now
                if self.debug_level >= 1:
                    print(f"Reconnecting to {self.writer.port}...")
                self.writer.reconnect()

            if now >= next_cycle:
                next_cycle += interval
                if next_cycle < now:          # we fell behind (e.g. laptop sleep)
                    next_cycle = now + interval
                fd = self.receiver.snapshot()
                # Only send while X-Plane is alive: frozen data would look like a real fix
                if self.receiver.is_connected():
                    sentences = self.converter.create_sentences(fd)
                    if sentences and self.writer.send_batch(sentences):
                        self.sentences_count += len(sentences)
                        self.last_sentences = sentences

            if self.debug_level >= 1 and now - last_stats >= 10 and not self.dry_run:
                last_stats = now
                fd = self.receiver.snapshot()
                xp = "OK" if self.receiver.is_connected() else "WAITING"
                out = "OK" if self.writer.is_connected() else "DISCONNECTED"
                vario = f", vario {fd.vario_ms:+.1f} m/s ({fd.vario_source})" \
                    if fd.vario_ms is not None else ""
                print(f"X-Plane:{xp} {self.writer.port}:{out} | "
                      f"{self.sentences_count} sentences sent{vario}")

            time.sleep(0.01)

    def get_status(self) -> dict:
        fd = self.receiver.snapshot()
        runtime = time.time() - self.start_time if self.running else 0.0
        return {
            "running": self.running,
            "xp_connected": self.receiver.is_connected(),
            "paused": fd.paused,
            "output_connected": self.writer.is_connected(),
            "output_port": self.writer.port,
            "flight": fd,
            "sentences_count": self.sentences_count,
            "sentences_rate": self.sentences_count / runtime if runtime > 0 else 0.0,
            "uptime": runtime,
        }

# ── GUI ───────────────────────────────────────────────────────────────

class BridgeGUI:
    """Tkinter window: X-Plane address, COM port, live flight data and status."""

    UPDATE_MS = 250
    FONT_LABEL = ("Arial", 10)
    FONT_VALUE = ("Consolas", 11, "bold")
    FONT_STATUS = ("Arial", 10, "bold")
    FONT_FOOTER = ("Consolas", 9)
    COLOR_GREEN = "#2E7D32"
    COLOR_RED = "#C62828"
    COLOR_AMBER = "#EF6C00"
    COLOR_GREY = "#757575"
    COLOR_BG = "#f5f5f5"

    def __init__(self, bridge: XPlaneToSeeYou):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.bridge = bridge
        self.root = tk.Tk()
        self.root.title(f"X-Plane 12 → SeeYou Navigator v{VERSION}")
        self.root.geometry("470x610")
        self.root.resizable(False, False)
        self.root.configure(bg=self.COLOR_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._port_info: Dict[str, str] = {}
        self._build_ui()

    def _build_ui(self):
        tk, ttk, bg = self.tk, self.ttk, self.COLOR_BG

        # ── Connection settings ──
        cfg = tk.LabelFrame(self.root, text=" Connection ", font=self.FONT_LABEL,
                            bg=bg, padx=10, pady=6)
        cfg.pack(fill="x", padx=10, pady=(8, 4))

        row_xp = tk.Frame(cfg, bg=bg)
        row_xp.pack(fill="x", pady=(0, 4))
        tk.Label(row_xp, text="X-Plane IP:", font=self.FONT_LABEL, bg=bg).pack(side="left")
        self.entry_host = tk.Entry(row_xp, width=16, font=self.FONT_LABEL)
        self.entry_host.insert(0, self.bridge.xp_host)
        self.entry_host.pack(side="left", padx=(4, 6))
        tk.Label(row_xp, text="(127.0.0.1 = this PC)", font=("Arial", 8),
                 fg=self.COLOR_GREY, bg=bg).pack(side="left")

        row_com = tk.Frame(cfg, bg=bg)
        row_com.pack(fill="x")
        tk.Label(row_com, text="Port:", font=self.FONT_LABEL, bg=bg).pack(side="left")
        self.combo_port = ttk.Combobox(row_com, width=8, state="readonly")
        self.combo_port.pack(side="left", padx=(4, 10))
        self.combo_port.bind("<<ComboboxSelected>>", lambda _e: self._show_port_info())
        tk.Label(row_com, text="Baud:", font=self.FONT_LABEL, bg=bg).pack(side="left")
        self.combo_baud = ttk.Combobox(row_com, width=7, state="readonly",
                                       values=[str(b) for b in BAUD_RATES])
        self.combo_baud.set(str(self.bridge.baudrate))
        self.combo_baud.pack(side="left", padx=(4, 10))
        tk.Button(row_com, text="Refresh", font=("Arial", 9),
                  command=self._refresh_ports).pack(side="right")

        self.lbl_port_info = tk.Label(cfg, text="", font=("Arial", 8), fg=self.COLOR_GREY,
                                      bg=bg, anchor="w")
        self.lbl_port_info.pack(fill="x", pady=(3, 0))
        self._refresh_ports()

        # ── Status ──
        status = tk.Frame(self.root, bg=bg)
        status.pack(fill="x", padx=10, pady=(4, 2))
        self.lbl_xp = tk.Label(status, text="● X-Plane: ---", font=self.FONT_STATUS,
                               fg=self.COLOR_GREY, bg=bg, anchor="w")
        self.lbl_xp.pack(side="left", expand=True, fill="x")
        self.lbl_serial = tk.Label(status, text="● Serial: ---", font=self.FONT_STATUS,
                                   fg=self.COLOR_GREY, bg=bg, anchor="e")
        self.lbl_serial.pack(side="right", expand=True, fill="x")

        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        # ── Flight data ──
        data = tk.LabelFrame(self.root, text=" Flight Data ", font=self.FONT_LABEL,
                             bg=bg, padx=10, pady=6)
        data.pack(fill="both", expand=True, padx=10, pady=4)
        self.data_labels = {}
        fields = [
            ("altitude", "Altitude"), ("pressure", "Press. alt"), ("agl", "AGL"),
            ("speed", "Ground spd"), ("ias_tas", "IAS / TAS"), ("heading", "Heading"),
            ("track", "Track"), ("vario", "Vario"), ("avg_vario", "Avg 30 s"), ("wind", "Wind"),
        ]
        for i, (key, label) in enumerate(fields):
            tk.Label(data, text=f"{label}:", font=self.FONT_LABEL, bg=bg, anchor="w",
                     width=10).grid(row=i, column=0, sticky="w", pady=1)
            value = tk.Label(data, text="---", font=self.FONT_VALUE, bg=bg, anchor="w")
            value.grid(row=i, column=1, sticky="w", padx=(8, 0), pady=1)
            self.data_labels[key] = value

        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        footer = tk.Frame(self.root, bg=bg)
        footer.pack(fill="x", padx=10, pady=2)
        self.lbl_nmea = tk.Label(footer, text="NMEA: 0 sent", font=self.FONT_FOOTER,
                                 bg=bg, anchor="w")
        self.lbl_nmea.pack(side="left")
        self.lbl_uptime = tk.Label(footer, text="Uptime: 00:00:00", font=self.FONT_FOOTER,
                                   bg=bg, anchor="e")
        self.lbl_uptime.pack(side="right")

        buttons = tk.Frame(self.root, bg=bg)
        buttons.pack(fill="x", padx=10, pady=(4, 10))
        self.btn_start = tk.Button(buttons, text="Start", font=self.FONT_LABEL,
                                   bg=self.COLOR_GREEN, fg="white", width=12,
                                   command=self._on_start)
        self.btn_start.pack(side="left", expand=True, padx=4)
        self.btn_stop = tk.Button(buttons, text="Stop", font=self.FONT_LABEL,
                                  bg=self.COLOR_RED, fg="white", width=12,
                                  command=self._on_stop, state="disabled")
        self.btn_stop.pack(side="right", expand=True, padx=4)

    def _refresh_ports(self):
        ports = SerialWriter.list_ports_detailed()
        self._port_info = {device: description for device, description, _, _ in ports}
        names = list(self._port_info)
        self.combo_port["values"] = names
        dongle = SerialWriter.find_naviter_dongle()
        if self.bridge.com_port in names:
            self.combo_port.set(self.bridge.com_port)
        elif dongle:
            self.combo_port.set(dongle)
        elif names:
            self.combo_port.set(names[0])
        else:
            self.combo_port.set("")
        self._show_port_info()

    def _show_port_info(self):
        port = self.combo_port.get()
        if not port:
            self.lbl_port_info.config(text="No COM ports found. Is the Naviter dongle plugged in?")
            return
        description = self._port_info.get(port, "")
        hint = "  ← looks like the Naviter dongle" if "CP210" in description else ""
        self.lbl_port_info.config(text=f"{description}{hint}")

    def _set_inputs_state(self, enabled: bool):
        self.entry_host.config(state="normal" if enabled else "disabled")
        self.combo_port.config(state="readonly" if enabled else "disabled")
        self.combo_baud.config(state="readonly" if enabled else "disabled")

    def _on_start(self):
        host = self.entry_host.get().strip() or DEFAULT_XP_HOST
        port = self.combo_port.get()
        self.bridge.xp_host = host
        if port:
            self.bridge.com_port = port
        self.bridge.baudrate = int(self.combo_baud.get())
        save_settings({"xp_host": host, "com_port": self.bridge.com_port,
                       "baudrate": self.bridge.baudrate})
        self.bridge.start_services()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self._set_inputs_state(False)
        self._schedule_update()

    def _on_stop(self):
        self.bridge.stop()
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self._set_inputs_state(True)
        self._reset_display()

    def _on_close(self):
        if self.bridge.running:
            self.bridge.stop()
        self.root.destroy()

    def _schedule_update(self):
        if not self.bridge.running:
            return
        self._update_display()
        self.root.after(self.UPDATE_MS, self._schedule_update)

    def _update_display(self):
        s = self.bridge.get_status()
        fd: FlightData = s["flight"]

        if not s["xp_connected"]:
            self.lbl_xp.config(text="● X-Plane: Waiting...", fg=self.COLOR_RED)
        elif s["paused"]:
            self.lbl_xp.config(text="● X-Plane: Paused", fg=self.COLOR_AMBER)
        else:
            self.lbl_xp.config(text="● X-Plane: Connected", fg=self.COLOR_GREEN)
        if s["output_connected"]:
            self.lbl_serial.config(text=f"● Serial: {s['output_port']}", fg=self.COLOR_GREEN)
        else:
            self.lbl_serial.config(text="● Serial: Disconnected", fg=self.COLOR_RED)

        d = self.data_labels
        if not fd.has_position:
            for label in d.values():
                label.config(text="---")
        else:
            d["altitude"].config(text=f"{fd.altitude_msl_m:.0f} m  ({fd.altitude_msl_m / FT_TO_M:.0f} ft) MSL")
            d["pressure"].config(text=self._fmt_alt(fd.pressure_altitude_m))
            d["agl"].config(text=self._fmt_alt(fd.agl_m))
            d["speed"].config(text=f"{fd.ground_speed_ms * MS_TO_KPH:.0f} km/h  "
                                   f"({fd.ground_speed_ms * MS_TO_KTS:.0f} kts)")
            if fd.ias_ms is not None and fd.tas_ms is not None:
                d["ias_tas"].config(text=f"{fd.ias_ms * MS_TO_KPH:.0f} / {fd.tas_ms * MS_TO_KPH:.0f} km/h")
            else:
                d["ias_tas"].config(text="---")
            d["heading"].config(text=f"{fd.heading_deg:.0f}° T")
            d["track"].config(text=f"{fd.track_deg:.0f}° T")
            if fd.vario_ms is not None:
                arrow = "▲" if fd.vario_ms > 0.1 else ("▼" if fd.vario_ms < -0.1 else "─")
                d["vario"].config(text=f"{fd.vario_ms:+.1f} m/s  {arrow}  ({fd.vario_source})")
            else:
                d["vario"].config(text="---")
            d["avg_vario"].config(text=f"{fd.vario_avg_ms:+.1f} m/s"
                                  if fd.vario_avg_ms is not None else "---")
            if fd.wind_dir_deg is not None and fd.wind_speed_ms is not None:
                d["wind"].config(text=f"{fd.wind_dir_deg:03.0f}° / {fd.wind_speed_ms * MS_TO_KPH:.0f} km/h")
            else:
                d["wind"].config(text="---")

        self.lbl_nmea.config(text=f"NMEA: {s['sentences_count']} sent ({s['sentences_rate']:.1f}/s)")
        up = int(s["uptime"])
        self.lbl_uptime.config(text=f"Uptime: {up // 3600:02d}:{up % 3600 // 60:02d}:{up % 60:02d}")

    @staticmethod
    def _fmt_alt(meters: Optional[float]) -> str:
        if meters is None:
            return "---"
        return f"{meters:.0f} m  ({meters / FT_TO_M:.0f} ft)"

    def _reset_display(self):
        self.lbl_xp.config(text="● X-Plane: ---", fg=self.COLOR_GREY)
        self.lbl_serial.config(text="● Serial: ---", fg=self.COLOR_GREY)
        for label in self.data_labels.values():
            label.config(text="---")
        self.lbl_nmea.config(text="NMEA: 0 sent")
        self.lbl_uptime.config(text="Uptime: 00:00:00")

    def run(self):
        self.root.mainloop()

# ── Main ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="X-Plane 12 to SeeYou Navigator bridge via serial COM port (Naviter dongle)")
    parser.add_argument("--xp-host", help=f"IP of the X-Plane PC (default: {DEFAULT_XP_HOST})")
    parser.add_argument("--xp-port", type=int, default=DEFAULT_XP_PORT,
                        help=f"X-Plane UDP receive port (default: {DEFAULT_XP_PORT})")
    parser.add_argument("--rref-freq", type=int, default=DEFAULT_RREF_FREQ,
                        help=f"Dataref rate requested from X-Plane in Hz (default: {DEFAULT_RREF_FREQ})")
    parser.add_argument("--com-port", help=f"Serial port of the Naviter dongle "
                        f"(default: last used, auto-detected CP210x, or {DEFAULT_COM_PORT})")
    parser.add_argument("--baudrate", type=int, choices=BAUD_RATES,
                        help=f"Serial baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--update-rate", type=float, default=DEFAULT_UPDATE_RATE,
                        help=f"NMEA output rate in Hz (default: {DEFAULT_UPDATE_RATE:g}, like Condor 3)")
    parser.add_argument("--baro-alt", choices=BARO_MODES, default="msl",
                        help="LXWP0 altitude: 'msl' true altitude like Condor 3 (default), "
                             "or 'pressure' altitude (QNE) like a real LX vario")
    parser.add_argument("--time-source", choices=TIME_SOURCES, default="sim",
                        help="NMEA UTC time: 'sim' X-Plane zulu time like Condor 3 (default), "
                             "or 'system' PC clock")
    parser.add_argument("--vario", choices=VARIO_MODES, default="auto",
                        help="Vario sent to Navigator: 'te' total energy, 'raw' vertical speed, "
                             "'auto' TE with fallback to raw (default)")
    parser.add_argument("--debug", type=int, default=DEFAULT_DEBUG_LEVEL, choices=[0, 1, 2],
                        help=f"Debug level (default: {DEFAULT_DEBUG_LEVEL})")
    parser.add_argument("--no-gui", action="store_true", help="Run in terminal mode")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print NMEA to the console instead of a COM port (implies --no-gui)")
    parser.add_argument("--list-ports", action="store_true", help="List COM ports and exit")
    args = parser.parse_args()

    if not HAS_PYSERIAL and not args.dry_run:
        print("ERROR: pyserial is required. Install with: pip install pyserial")
        sys.exit(1)

    if args.list_ports:
        ports = SerialWriter.list_ports_detailed()
        if not ports:
            print("No COM ports found.")
        for device, description, _, _ in ports:
            hint = "  <- Naviter dongle?" if "CP210" in description else ""
            print(f"  {device}: {description}{hint}")
        sys.exit(0)

    saved = load_settings()
    com_port = args.com_port or saved.get("com_port")
    if not com_port:
        com_port = (SerialWriter.find_naviter_dongle() if HAS_PYSERIAL else None) or DEFAULT_COM_PORT

    bridge = XPlaneToSeeYou(
        xp_host=args.xp_host or saved.get("xp_host") or DEFAULT_XP_HOST,
        xp_port=args.xp_port,
        com_port=com_port,
        baudrate=args.baudrate or saved.get("baudrate") or DEFAULT_BAUDRATE,
        update_rate=args.update_rate,
        rref_freq=args.rref_freq,
        baro_mode=args.baro_alt,
        time_source=args.time_source,
        vario_mode=args.vario,
        debug_level=args.debug,
        dry_run=args.dry_run,
    )

    if args.no_gui or args.dry_run:
        bridge.start()
    else:
        BridgeGUI(bridge).run()


if __name__ == "__main__":
    main()
