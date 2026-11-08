"""XPPython3 plugin: X-Plane 12 to SeeYou Navigator through the Naviter dongle.

Reads the datarefs inside X-Plane (doubles for position, no UDP), builds the
same Condor 3 NMEA as the standalone app and writes it to the COM port from a
background thread, so the simulator frame never waits for the serial port.

Plugins menu > SeeYou Navigator: output on/off, status window, COM port, baud
rate, vario source and LXWP0 altitude. Settings are saved in X-Plane's
Output/preferences/xp12_seeyou_navigator.json.
"""

import json
import os
import queue
import time
from typing import Dict, List, Optional, Tuple

from XPPython3 import xp

from . import VERSION
from .core import (
    BARO_MODES, BAUD_RATES, DATAREFS, DEFAULT_BAUDRATE, DEFAULT_COM_PORT,
    DEFAULT_UPDATE_RATE, MS_TO_KPH, TIME_SOURCES, VARIO_MODES,
    FlightData, NMEAConverter, VarioTracker, build_flight_data,
)
from .serial_out import (
    HAS_PYSERIAL, ThreadedSerialWriter, find_naviter_dongle, is_naviter_dongle,
    list_ports_detailed,
)

PLUGIN_NAME = "SeeYou Navigator Bridge"
PLUGIN_SIG = "jlgabriel.xp12.seeyou_navigator"
PLUGIN_DESC = "Sends flight data to SeeYou Navigator through the Naviter dongle (Condor 3 NMEA)."
MENU_TITLE = "SeeYou Navigator"
SETTINGS_FILE = "xp12_seeyou_navigator.json"

SAMPLE_INTERVAL = 0.1        # seconds between dataref reads (vario averaging)
WINDOW_SIZE = (430, 190)

WHITE = (1.0, 1.0, 1.0)
GREEN = (0.45, 1.0, 0.45)
RED = (1.0, 0.45, 0.45)
AMBER = (1.0, 0.75, 0.3)
GREY = (0.7, 0.7, 0.7)

DEFAULT_SETTINGS = {
    "enabled": True,
    "com_port": None,          # None: Naviter dongle if found, else COM7
    "baudrate": DEFAULT_BAUDRATE,
    "update_rate": DEFAULT_UPDATE_RATE,
    "baro_mode": "msl",
    "vario_mode": "auto",
    "time_source": "sim",
    "show_window": False,
}

VARIO_LABELS = {"auto": "Auto (TE, raw if no TE)", "te": "Total energy (TE)", "raw": "Raw vertical speed"}
BARO_LABELS = {"msl": "True MSL (like Condor 3)", "pressure": "Pressure altitude (QNE)"}


def log(message: str) -> None:
    xp.log(f"[SeeYou] {message}")


def settings_path() -> str:
    return os.path.join(os.path.dirname(xp.getPrefsPath()), SETTINGS_FILE)


def load_settings() -> dict:
    settings = dict(DEFAULT_SETTINGS)
    try:
        with open(settings_path(), "r", encoding="utf-8") as f:
            saved = json.load(f)
        if isinstance(saved, dict):
            settings.update({k: v for k, v in saved.items() if k in DEFAULT_SETTINGS})
    except (OSError, ValueError):
        pass
    # Repair values a hand-edited file could break
    if settings["baudrate"] not in BAUD_RATES:
        settings["baudrate"] = DEFAULT_BAUDRATE
    if settings["baro_mode"] not in BARO_MODES:
        settings["baro_mode"] = "msl"
    if settings["vario_mode"] not in VARIO_MODES:
        settings["vario_mode"] = "auto"
    if settings["time_source"] not in TIME_SOURCES:
        settings["time_source"] = "sim"
    try:
        settings["update_rate"] = max(0.2, min(float(settings["update_rate"]), 5.0))
    except (TypeError, ValueError):
        settings["update_rate"] = DEFAULT_UPDATE_RATE
    return settings


def save_settings(settings: dict) -> None:
    try:
        with open(settings_path(), "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=2)
    except OSError as e:
        log(f"could not save settings: {e}")


class PythonInterface:
    """XPPython3 entry point (re-exported by PI_xp12_seeyou_navigator.py)."""

    def __init__(self):
        self.clock = time.monotonic
        self.settings: dict = dict(DEFAULT_SETTINGS)
        self.refs: Dict[str, Tuple[object, str]] = {}
        self.values: Dict[str, float] = {}
        self.vario = VarioTracker("auto")
        self.converter = NMEAConverter()
        self.writer: Optional[ThreadedSerialWriter] = None
        self.flight = FlightData()
        self.last_sentences: List[str] = []
        self.last_output = 0.0
        self.loop_id = None
        self.menu_id = None
        self.window_id = None
        self._menu_items: Dict[tuple, Tuple[object, int]] = {}
        self._submenus: Dict[str, object] = {}
        self._last_loop_error: Optional[str] = None
        self._thread_logs: "queue.SimpleQueue[str]" = queue.SimpleQueue()

    # ── XPPython3 callbacks ──────────────────────────────────────────

    def XPluginStart(self):
        return PLUGIN_NAME, PLUGIN_SIG, PLUGIN_DESC

    def XPluginEnable(self):
        self.settings = load_settings()
        self.refs, missing = {}, []
        for key, path, kind in DATAREFS:
            ref = xp.findDataRef(path)
            if ref is None:
                missing.append(path)
            else:
                self.refs[key] = (ref, kind)
        if missing:
            log(f"datarefs not found (ignored): {', '.join(missing)}")

        self.values = {}
        self.vario = VarioTracker(self.settings["vario_mode"])
        self.converter = NMEAConverter(self.settings["baro_mode"], self.settings["time_source"])
        self.flight = FlightData()
        self.last_sentences = []
        self.last_output = 0.0

        if not HAS_PYSERIAL:
            log("pyserial is not installed in XPPython3's Python: "
                "XPPython3 menu > Pip Package Installer > pyserial, then restart X-Plane")
        if self.settings["enabled"]:
            self._start_output()

        self.loop_id = xp.createFlightLoop(self._flight_loop, xp.FlightLoop_Phase_AfterFlightModel)
        xp.scheduleFlightLoop(self.loop_id, SAMPLE_INTERVAL, 1)
        self._create_window()
        self._build_menu()
        log(f"v{VERSION} enabled")
        return 1

    def XPluginDisable(self):
        if self.loop_id is not None:
            xp.destroyFlightLoop(self.loop_id)
            self.loop_id = None
        self._stop_output()
        if self.window_id is not None:
            xp.destroyWindow(self.window_id)
            self.window_id = None
        if self.menu_id is not None:
            xp.destroyMenu(self.menu_id)
            self.menu_id = None
        self._menu_items, self._submenus = {}, {}
        save_settings(self.settings)

    def XPluginStop(self):
        pass

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass

    # ── Flight loop ──────────────────────────────────────────────────

    def _flight_loop(self, sinceLast, elapsedTime, counter, refCon):
        self._flush_thread_logs()
        try:
            now = self.clock()
            self._sample(now)
            if now - self.last_output >= 1.0 / self.settings["update_rate"]:
                self.last_output = now
                self.flight = build_flight_data(self.values, time.time(), self.vario)
                if self.writer is not None:
                    sentences = self.converter.create_sentences(self.flight)
                    if sentences:
                        self.writer.submit(sentences)
                        self.last_sentences = sentences
            self._last_loop_error = None
        except Exception as e:  # log once per distinct error, keep the loop alive
            message = f"{type(e).__name__}: {e}"
            if message != self._last_loop_error:
                log(f"flight loop error: {message}")
                self._last_loop_error = message
        return SAMPLE_INTERVAL

    def _sample(self, now: float):
        for key, (ref, kind) in self.refs.items():
            if kind == "d":
                self.values[key] = xp.getDatad(ref)
            elif kind == "i":
                self.values[key] = xp.getDatai(ref)
            else:
                self.values[key] = xp.getDataf(ref)
        self.vario.add_from_values(now, self.values)

    # ── Output ───────────────────────────────────────────────────────

    def make_writer(self, port: str, baudrate: int) -> ThreadedSerialWriter:
        # The writer thread must not call xp.*: its messages are queued and
        # written to the log by the flight loop, on X-Plane's thread.
        return ThreadedSerialWriter(port, baudrate, log=self._thread_logs.put)

    def _flush_thread_logs(self):
        while True:
            try:
                log(self._thread_logs.get_nowait())
            except queue.Empty:
                return

    def _start_output(self):
        self._stop_output()
        if not self.settings["com_port"]:
            self.settings["com_port"] = find_naviter_dongle() or DEFAULT_COM_PORT
        self.writer = self.make_writer(self.settings["com_port"], self.settings["baudrate"])
        self.writer.start()
        log(f"output to {self.settings['com_port']} @ {self.settings['baudrate']} baud")

    def _stop_output(self):
        if self.writer is not None:
            self.writer.stop()
            self.writer = None
            self._flush_thread_logs()
            log("output stopped")

    # ── Menu ─────────────────────────────────────────────────────────

    def _build_menu(self):
        self.menu_id = xp.createMenu(MENU_TITLE, None, 0, self._menu_handler, None)
        self._menu_items, self._submenus = {}, {}
        self._add_item(self.menu_id, "Send to SeeYou Navigator", ("toggle",))
        self._add_item(self.menu_id, "Show status window", ("window",))
        xp.appendMenuSeparator(self.menu_id)

        for name, title in (("port", "COM port"), ("baud", "Baud rate"),
                            ("vario", "Vario"), ("baro", "LXWP0 altitude")):
            index = xp.appendMenuItem(self.menu_id, title, None)
            self._submenus[name] = xp.createMenu(title, self.menu_id, index, self._menu_handler, None)

        self._fill_port_menu()
        for baud in BAUD_RATES:
            self._add_item(self._submenus["baud"], str(baud), ("baud", baud))
        for mode in VARIO_MODES:
            self._add_item(self._submenus["vario"], VARIO_LABELS[mode], ("vario", mode))
        for mode in BARO_MODES:
            self._add_item(self._submenus["baro"], BARO_LABELS[mode], ("baro", mode))
        self._refresh_checks()

    def _add_item(self, menu_id, name: str, ref: tuple) -> int:
        index = xp.appendMenuItem(menu_id, name, ref)
        self._menu_items[ref] = (menu_id, index)
        return index

    def _fill_port_menu(self):
        menu = self._submenus["port"]
        xp.clearAllMenuItems(menu)
        self._menu_items = {k: v for k, v in self._menu_items.items() if k[0] not in ("port", "refresh")}
        ports = list_ports_detailed()
        devices = [p[0] for p in ports]
        for device, description, vid, pid in ports:
            label = f"{device}  - Naviter dongle" if is_naviter_dongle(description, vid, pid) \
                else f"{device}  - {description[:40]}"
            self._add_item(menu, label, ("port", device))
        current = self.settings["com_port"]
        if current and current not in devices:
            self._add_item(menu, f"{current}  - not connected", ("port", current))
        if not ports and not current:
            self._add_item(menu, "(no COM ports found)", ("none",))
        xp.appendMenuSeparator(menu)
        self._add_item(menu, "Refresh list", ("refresh",))

    def _refresh_checks(self):
        s = self.settings
        selected = {
            "toggle": bool(s["enabled"]),
            "window": self.window_id is not None and bool(xp.getWindowIsVisible(self.window_id)),
        }
        for ref, (menu_id, index) in self._menu_items.items():
            kind = ref[0]
            if kind in selected:
                checked = selected[kind]
            elif kind == "port":
                checked = ref[1] == s["com_port"]
            elif kind == "baud":
                checked = ref[1] == s["baudrate"]
            elif kind == "vario":
                checked = ref[1] == s["vario_mode"]
            elif kind == "baro":
                checked = ref[1] == s["baro_mode"]
            else:
                continue
            xp.checkMenuItem(menu_id, index, xp.Menu_Checked if checked else xp.Menu_Unchecked)

    def _menu_handler(self, menuRef, itemRef):
        if not isinstance(itemRef, tuple) or not itemRef:
            return
        kind = itemRef[0]
        s = self.settings
        if kind == "toggle":
            s["enabled"] = not s["enabled"]
            if s["enabled"]:
                self._start_output()
            else:
                self._stop_output()
        elif kind == "window":
            if self.window_id is not None:
                visible = not xp.getWindowIsVisible(self.window_id)
                xp.setWindowIsVisible(self.window_id, 1 if visible else 0)
                s["show_window"] = visible
        elif kind in ("port", "baud"):
            if kind == "port":
                s["com_port"] = itemRef[1]
            else:
                s["baudrate"] = itemRef[1]
            if s["enabled"]:
                self._start_output()   # reopen with the new port / speed
        elif kind == "vario":
            s["vario_mode"] = itemRef[1]
            self.vario.mode = itemRef[1]
        elif kind == "baro":
            s["baro_mode"] = itemRef[1]
            self.converter.baro_mode = itemRef[1]
        elif kind == "refresh":
            self._fill_port_menu()
        else:
            return
        self._refresh_checks()
        save_settings(s)

    # ── Status window ────────────────────────────────────────────────

    def _create_window(self):
        left, top, _, _ = xp.getScreenBoundsGlobal()
        width, height = WINDOW_SIZE
        x, y = left + 80, top - 150
        self.window_id = xp.createWindowEx(
            x, y, x + width, y - height, 1 if self.settings["show_window"] else 0,
            draw=self._draw_window,
            decoration=xp.WindowDecorationRoundRectangle,
            layer=xp.WindowLayerFloatingWindows,
        )
        xp.setWindowTitle(self.window_id, f"SeeYou Navigator v{VERSION}")
        xp.setWindowPositioningMode(self.window_id, xp.WindowPositionFree, -1)

    def _draw_window(self, windowID, refCon):
        left, top, _, _ = xp.getWindowGeometry(windowID)
        for i, (text, color) in enumerate(self.status_lines()):
            xp.drawString(color, left + 12, top - 22 - i * 19, text, None, xp.Font_Proportional)

    def status_lines(self) -> List[Tuple[str, Tuple[float, float, float]]]:
        """Text of the status window, one (text, colour) per line."""
        lines = []
        if not HAS_PYSERIAL:
            lines.append(("pyserial missing: XPPython3 > Pip Package Installer > pyserial", RED))
        elif self.writer is None:
            lines.append(("Output OFF (Plugins > SeeYou Navigator > Send...)", GREY))
        elif self.writer.is_connected():
            lines.append((f"Serial: {self.writer.port} @ {self.writer.baudrate} baud - sending", GREEN))
        else:
            reason = self.writer.last_error or "opening..."
            lines.append((f"Serial: {self.writer.port} - {reason}"[:70], RED))
        if self.writer is not None:
            lines.append((f"Sentences sent: {self.writer.sentences_sent}", WHITE))

        fd = self.flight
        if not fd.has_position:
            lines.append(("Waiting for flight data...", AMBER))
            return lines
        pressure = f"   Pressure alt: {fd.pressure_altitude_m:.0f} m" \
            if fd.pressure_altitude_m is not None else ""
        lines.append((f"Position: {fd.latitude:.5f}, {fd.longitude:.5f}"
                      f"{'   (PAUSED)' if fd.paused else ''}", AMBER if fd.paused else WHITE))
        lines.append((f"Altitude: {fd.altitude_msl_m:.0f} m MSL{pressure}", WHITE))
        if fd.vario_ms is not None:
            avg = f"   30 s avg: {fd.vario_avg_ms:+.1f}" if fd.vario_avg_ms is not None else ""
            lines.append((f"Vario: {fd.vario_ms:+.1f} m/s ({fd.vario_source}){avg}", WHITE))
        tas = f"TAS: {fd.tas_ms * MS_TO_KPH:.0f} km/h" if fd.tas_ms is not None else "TAS: ---"
        wind = f"   Wind: {fd.wind_dir_deg:03.0f} deg / {fd.wind_speed_ms * MS_TO_KPH:.0f} km/h" \
            if fd.wind_dir_deg is not None and fd.wind_speed_ms is not None else ""
        lines.append((tas + wind, WHITE))
        if self.last_sentences:
            lines.append((self.last_sentences[-1], GREY))
        return lines
