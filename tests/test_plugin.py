"""Tests of the XPPython3 plugin with a minimal fake `xp` module.

Not a simulation of X-Plane: it checks that the plugin reads the datarefs
(doubles for position), sends Condor 3 NMEA once per second from the flight
loop, reacts to its menu, draws its status window, and cleans up on disable.
"""

import importlib
import json
import os
import shutil
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from xp12_seeyou.core import NMEAConverter  # noqa: E402

DATA = {
    "sim/flightmodel/position/latitude": -33.391234567,
    "sim/flightmodel/position/longitude": -70.791234567,
    "sim/flightmodel/position/elevation": 1500.0,
    "sim/flightmodel2/position/pressure_altitude": 4600.0,          # ft
    "sim/flightmodel/position/y_agl": 1000.0,
    "sim/flightmodel/position/groundspeed": 25.0,
    "sim/flightmodel/position/local_vx": 25.0,                       # flying east
    "sim/flightmodel/position/local_vz": 0.0,
    "sim/flightmodel/position/true_psi": 88.0,
    "sim/flightmodel/position/magnetic_variation": 1.5,
    "sim/flightmodel/position/indicated_airspeed": 46.0,
    "sim/flightmodel/position/true_airspeed": 25.0,
    "sim/flightmodel/position/vh_ind": 1.8,
    "sim/cockpit2/gauges/indicators/total_energy_fpm": 300.0,        # 1.524 m/s
    "sim/weather/aircraft/wind_now_direction_degt": 240.0,
    "sim/weather/aircraft/wind_now_speed_msc": 5.0,
    "sim/time/zulu_time_sec": 61200.0,
    "sim/time/paused": 0,
}


def make_fake_xp(prefs_dir, data):
    xp = types.ModuleType("xp")
    xp.log_lines = []
    xp.menus = {}            # menu id -> list of [name, ref, check]
    xp.handlers = {}
    xp.windows = {}
    xp.drawn = []
    xp.loops = {}
    for name in ("FlightLoop_Phase_AfterFlightModel", "WindowDecorationRoundRectangle",
                 "WindowLayerFloatingWindows", "WindowPositionFree", "Font_Proportional"):
        setattr(xp, name, name)
    xp.Menu_Unchecked, xp.Menu_Checked = 1, 2

    xp.log = xp.log_lines.append
    xp.getPrefsPath = lambda: os.path.join(prefs_dir, "X-Plane.prf")
    xp.findDataRef = lambda path: path if path in data else None
    xp.getDatad = lambda ref: float(data[ref])
    xp.getDataf = lambda ref: float(data[ref])
    xp.getDatai = lambda ref: int(data[ref])

    def create_flight_loop(callback, phase, refCon=None):
        loop_id = f"loop{len(xp.loops)}"
        xp.loops[loop_id] = callback
        return loop_id
    xp.createFlightLoop = create_flight_loop
    xp.scheduleFlightLoop = lambda loop_id, interval, relative: None
    xp.destroyFlightLoop = lambda loop_id: xp.loops.pop(loop_id)

    def create_menu(name, parent, item, handler, refCon):
        menu_id = f"menu{len(xp.handlers)}:{name}"
        xp.menus[menu_id] = []
        xp.handlers[menu_id] = handler
        return menu_id
    xp.createMenu = create_menu

    def append_item(menu_id, name, ref=None):
        xp.menus[menu_id].append([name, ref, 0])
        return len(xp.menus[menu_id]) - 1
    xp.appendMenuItem = append_item
    xp.appendMenuSeparator = lambda menu_id: append_item(menu_id, "-", None)
    xp.clearAllMenuItems = lambda menu_id: xp.menus[menu_id].clear()
    xp.destroyMenu = lambda menu_id: xp.menus.pop(menu_id)

    def check_item(menu_id, index, state):
        xp.menus[menu_id][index][2] = state
    xp.checkMenuItem = check_item

    xp.getScreenBoundsGlobal = lambda: (0, 1080, 1920, 0)

    def create_window(left, top, right, bottom, visible, draw=None, **kwargs):
        window_id = f"win{len(xp.windows)}"
        xp.windows[window_id] = {"visible": visible, "draw": draw, "geometry": (left, top, right, bottom)}
        return window_id
    xp.createWindowEx = create_window
    xp.setWindowTitle = lambda window_id, title: None
    xp.setWindowPositioningMode = lambda window_id, mode, index: None
    xp.getWindowGeometry = lambda window_id: xp.windows[window_id]["geometry"]
    xp.getWindowIsVisible = lambda window_id: xp.windows[window_id]["visible"]
    xp.setWindowIsVisible = lambda window_id, v: xp.windows[window_id].__setitem__("visible", v)
    xp.destroyWindow = lambda window_id: xp.windows.pop(window_id)
    xp.drawString = lambda color, x, y, text, wrap, font: xp.drawn.append(text)
    return xp


class FakeWriter:
    def __init__(self, port, baudrate):
        self.port, self.baudrate = port, baudrate
        self.batches, self.started, self.stopped = [], False, False
        self.last_error = None
        self.sentences_sent = 0

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def is_connected(self):
        return self.started and not self.stopped

    def submit(self, sentences):
        self.batches.append(list(sentences))
        self.sentences_sent += len(sentences)


class TestPlugin(unittest.TestCase):

    def setUp(self):
        self.prefs = tempfile.mkdtemp()
        self.data = dict(DATA)
        self.xp = make_fake_xp(self.prefs, self.data)
        package = types.ModuleType("XPPython3")
        package.xp = self.xp
        sys.modules["XPPython3"] = package
        sys.modules.pop("xp12_seeyou.plugin", None)
        self.mod = importlib.import_module("xp12_seeyou.plugin")
        self.mod.HAS_PYSERIAL = True
        self.mod.list_ports_detailed = lambda: [
            ("COM1", "Puerto de comunicaciones (COM1)", None, None),
            ("COM7", "Silicon Labs CP210x USB to UART Bridge (COM7)", 0x10C4, 0xEA60),
        ]
        self.mod.find_naviter_dongle = lambda: "COM7"
        self.writers = []

        test = self

        class Plugin(self.mod.PythonInterface):
            def make_writer(self, port, baudrate):
                writer = FakeWriter(port, baudrate)
                test.writers.append(writer)
                return writer

        self.plugin = Plugin()
        self.now = 100.0
        self.plugin.clock = lambda: self.now
        self.assertEqual(self.plugin.XPluginStart()[1], self.mod.PLUGIN_SIG)
        self.assertEqual(self.plugin.XPluginEnable(), 1)

    def tearDown(self):
        if self.plugin.loop_id is not None:
            self.plugin.XPluginDisable()
        sys.modules.pop("XPPython3", None)
        sys.modules.pop("xp12_seeyou.plugin", None)
        shutil.rmtree(self.prefs, ignore_errors=True)

    # helpers ---------------------------------------------------------

    def run_loop(self, seconds):
        callback = self.xp.loops[self.plugin.loop_id]
        for _ in range(int(round(seconds / 0.1))):
            self.assertEqual(callback(0.1, self.now, 0, None), self.mod.SAMPLE_INTERVAL)
            self.now += 0.1

    def click(self, ref):
        for menu_id, items in self.xp.menus.items():
            for name, item_ref, _ in items:
                if item_ref == ref:
                    self.xp.handlers[menu_id](None, item_ref)
                    return
        self.fail(f"menu item {ref} not found")

    def checked(self, ref):
        for items in self.xp.menus.values():
            for name, item_ref, state in items:
                if item_ref == ref:
                    return state == self.xp.Menu_Checked
        self.fail(f"menu item {ref} not found")

    def saved_settings(self):
        with open(os.path.join(self.prefs, self.mod.SETTINGS_FILE), encoding="utf-8") as f:
            return json.load(f)

    # tests -----------------------------------------------------------

    def test_sends_condor_nmea_once_per_second(self):
        writer = self.writers[-1]
        self.assertEqual((writer.port, writer.baudrate), ("COM7", 4800))   # dongle auto-detected
        self.run_loop(3.0)
        self.assertEqual(len(writer.batches), 3)
        gga, rmc, lx = writer.batches[-1]
        lat, ns = NMEAConverter.latitude_to_nmea(DATA["sim/flightmodel/position/latitude"])
        self.assertEqual(gga.split(",")[2:4], [lat, ns])        # double precision kept
        self.assertEqual(gga.split(",")[1], "170000.000")       # sim zulu time
        self.assertEqual(rmc.split(",")[8], "90.00")            # track from velocity, east
        fields = lx.split("*")[0].split(",")
        self.assertEqual(fields[2], "90.0")                     # TAS 25 m/s
        self.assertEqual(fields[3], "1500.0")                   # true MSL by default
        self.assertEqual(fields[4], "1.52")                     # TE vario 300 fpm
        self.assertEqual(fields[10:], ["88", "240", "18.0"])

    def test_toggle_output_from_menu(self):
        self.assertTrue(self.checked(("toggle",)))
        self.click(("toggle",))
        self.assertTrue(self.writers[-1].stopped)
        self.assertIsNone(self.plugin.writer)
        self.assertFalse(self.checked(("toggle",)))
        self.assertFalse(self.saved_settings()["enabled"])
        self.run_loop(2.0)                                      # no writer, no crash
        self.click(("toggle",))
        self.assertTrue(self.writers[-1].started)
        self.assertTrue(self.saved_settings()["enabled"])

    def test_port_and_baud_selection_reopen_output(self):
        port_items = [item[0] for item in self.xp.menus[self.plugin._submenus["port"]]]
        self.assertIn("COM7  - Naviter dongle", port_items)
        self.assertTrue(self.checked(("port", "COM7")))
        self.click(("port", "COM1"))
        self.assertEqual(self.writers[-1].port, "COM1")
        self.assertTrue(self.writers[-2].stopped)
        self.assertTrue(self.checked(("port", "COM1")))
        self.assertFalse(self.checked(("port", "COM7")))
        self.click(("baud", 9600))
        self.assertEqual((self.writers[-1].port, self.writers[-1].baudrate), ("COM1", 9600))
        self.assertEqual(self.saved_settings()["com_port"], "COM1")

    def test_pressure_altitude_and_raw_vario_options(self):
        self.click(("baro", "pressure"))
        self.click(("vario", "raw"))
        self.run_loop(1.5)
        fields = self.writers[-1].batches[-1][2].split(",")
        self.assertEqual(fields[3], f"{4600 * 0.3048:.1f}")
        self.assertEqual(fields[4], "1.80")
        self.assertTrue(self.checked(("baro", "pressure")))
        self.assertEqual(self.saved_settings()["vario_mode"], "raw")

    def test_settings_survive_restart(self):
        self.click(("port", "COM1"))
        self.click(("baro", "pressure"))
        self.plugin.XPluginDisable()
        self.plugin.XPluginEnable()
        self.assertEqual(self.writers[-1].port, "COM1")
        self.assertEqual(self.plugin.converter.baro_mode, "pressure")
        self.assertTrue(self.checked(("baro", "pressure")))

    def test_status_window(self):
        window_id = self.plugin.window_id
        self.assertFalse(self.xp.windows[window_id]["visible"])
        self.click(("window",))
        self.assertTrue(self.xp.windows[window_id]["visible"])
        self.assertTrue(self.checked(("window",)))
        self.run_loop(1.5)
        self.xp.windows[window_id]["draw"](window_id, None)
        text = "\n".join(self.xp.drawn)
        self.assertIn("Serial: COM7 @ 4800 baud - sending", text)
        self.assertIn("Vario: +1.5 m/s (TE)", text)
        self.assertIn("Wind: 240 deg / 18 km/h", text)
        self.assertIn("$LXWP0", text)

    def test_missing_te_dataref_falls_back_to_raw(self):
        self.plugin.XPluginDisable()
        del self.data["sim/cockpit2/gauges/indicators/total_energy_fpm"]
        self.plugin.XPluginEnable()
        self.assertTrue(any("total_energy_fpm" in line for line in self.xp.log_lines))
        self.run_loop(5.0)
        self.assertEqual(self.writers[-1].batches[-1][2].split(",")[4], "1.80")

    def test_paused_sim_still_sends_position(self):
        self.data["sim/time/paused"] = 1
        self.run_loop(1.5)
        self.assertEqual(len(self.writers[-1].batches), 2)
        self.assertTrue(self.plugin.flight.paused)

    def test_flight_loop_errors_are_logged_once(self):
        def broken(ref):
            raise RuntimeError("boom")
        self.xp.getDataf = broken
        self.run_loop(1.0)
        errors = [line for line in self.xp.log_lines if "boom" in line]
        self.assertEqual(len(errors), 1)

    def test_disable_cleans_up(self):
        loop_id, window_id, menu_id = self.plugin.loop_id, self.plugin.window_id, self.plugin.menu_id
        self.plugin.XPluginDisable()
        self.assertNotIn(loop_id, self.xp.loops)
        self.assertNotIn(window_id, self.xp.windows)
        self.assertNotIn(menu_id, self.xp.menus)
        self.assertTrue(self.writers[-1].stopped)
        self.assertIsNone(self.plugin.loop_id)


class TestThreadedSerialWriter(unittest.TestCase):

    def test_submit_never_blocks_and_writes_in_background(self):
        import threading
        from xp12_seeyou.serial_out import ThreadedSerialWriter

        release = threading.Event()

        class SlowWriter:
            port, baudrate, last_error = "COM7", 4800, None

            def __init__(self):
                self.written = []

            def is_connected(self):
                return True

            def reconnect(self):
                pass

            def send_batch(self, sentences):
                release.wait(2)          # a port that is slow to accept data
                self.written.append(sentences)
                return True

            def stop(self):
                pass

        slow = SlowWriter()
        writer = ThreadedSerialWriter(writer=slow)
        writer.start()
        import time
        start = time.monotonic()
        for i in range(10):
            writer.submit([f"$S{i}"])
        self.assertLess(time.monotonic() - start, 0.1)
        release.set()
        time.sleep(0.3)
        writer.stop()
        self.assertEqual(slow.written[-1], ["$S9"])            # newest cycle always kept
        self.assertLessEqual(len(slow.written), 4)             # backlog was trimmed


if __name__ == "__main__":
    unittest.main()
