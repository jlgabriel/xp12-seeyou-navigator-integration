"""Tests for the X-Plane 12 to SeeYou Navigator bridge. Run: python -m unittest discover tests"""

import importlib.util
import os
import re
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

_spec = importlib.util.spec_from_file_location(
    "bridge", os.path.join(ROOT, "xp12-seeyou-navigator-integration.py"))
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)

from fake_xplane import FakeXPlane, GliderModel  # noqa: E402

NMEA = bridge.NMEAConverter


def sample_flight(**overrides):
    values = dict(
        timestamp=time.time(), latitude=45.8895, longitude=13.890602, altitude_msl_m=117.5,
        pressure_altitude_m=80.0, ground_speed_ms=0.0, track_deg=267.8, heading_deg=268.0,
        tas_ms=17.5 / 3.6, vario_ms=0.0, vario_source="TE", wind_dir_deg=268.0,
        wind_speed_ms=0.0, zulu_sec=17 * 3600 + 0.021,
    )
    values.update(overrides)
    return bridge.FlightData(**values)


class ListWriter:
    RECONNECT_INTERVAL = 3.0
    port = "test"

    def __init__(self):
        self.batches = []

    def start(self):
        pass

    def stop(self):
        pass

    def reconnect(self):
        pass

    def is_connected(self):
        return True

    def send_batch(self, sentences):
        self.batches.append(list(sentences))
        return True


class TestNMEAFormat(unittest.TestCase):

    def test_checksums_match_condor3_capture(self):
        captured = [
            "$GPGGA,170000.021,4553.3700,N,01353.4361,E,1,12,10,117.5,M,,,,,0000*0F",
            "$GPRMC,170000.021,A,4553.3700,N,01353.4361,E,0.00,267.80,,,,*26",
            "$LXWP0,Y,17.5,117.5,0.00,,,,,,268,268,0.0*7B",
        ]
        for line in captured:
            body, checksum = line[1:].split("*")
            self.assertEqual(NMEA.checksum(body), checksum, line)

    def test_sentences_reproduce_condor3_capture(self):
        # Condor 3 capture, first cycle; position chosen to give the same NMEA minutes
        sentences = NMEA("msl", "sim").create_sentences(sample_flight())
        self.assertEqual(sentences, [
            "$GPGGA,170000.021,4553.3700,N,01353.4361,E,1,12,10,117.5,M,,,,,0000*0F",
            "$GPRMC,170000.021,A,4553.3700,N,01353.4361,E,0.00,267.80,,,,*26",
            "$LXWP0,Y,17.5,117.5,0.00,,,,,,268,268,0.0*7B",
        ])

    def test_coordinates(self):
        self.assertEqual(NMEA.latitude_to_nmea(-33.5), ("3330.0000", "S"))
        self.assertEqual(NMEA.longitude_to_nmea(-70.75), ("07045.0000", "W"))
        self.assertEqual(NMEA.longitude_to_nmea(5.0), ("00500.0000", "E"))

    def test_minutes_never_reach_60(self):
        self.assertEqual(NMEA.latitude_to_nmea(33.999999999), ("3400.0000", "N"))
        self.assertEqual(NMEA.longitude_to_nmea(-70.9999999999), ("07100.0000", "W"))

    def test_heading_rounds_to_0_not_360(self):
        lx = NMEA().create_sentences(sample_flight(heading_deg=359.7, wind_dir_deg=359.6))[2]
        self.assertEqual(lx.split(",")[10:12], ["0", "0"])

    def test_pressure_mode_uses_pressure_altitude(self):
        lx = NMEA("pressure").create_sentences(sample_flight())[2]
        self.assertEqual(lx.split(",")[3], "80.0")
        gga = NMEA("pressure").create_sentences(sample_flight())[0]
        self.assertEqual(gga.split(",")[9], "117.5")  # GGA stays true MSL

    def test_system_time_source(self):
        gga = NMEA("msl", "system").create_sentences(sample_flight())[0]
        self.assertRegex(gga.split(",")[1], r"^\d{6}\.\d{3}$")

    def test_no_position_no_sentences(self):
        self.assertEqual(NMEA().create_sentences(bridge.FlightData()), [])


class TestRREF(unittest.TestCase):

    def test_request_layout(self):
        packet = bridge.XPlaneReceiver.build_rref_request(10, 3, "sim/time/paused")
        self.assertEqual(len(packet), 413)
        self.assertEqual(packet[:5], b"RREF\x00")
        self.assertEqual(packet[13:28], b"sim/time/paused")
        self.assertEqual(packet[28:], b"\x00" * 385)

    def test_parse_answer(self):
        import struct
        data = b"RREF," + struct.pack("<if", 0, -33.5) + struct.pack("<if", 17, 1.0)
        self.assertEqual(bridge.XPlaneReceiver.parse_rref_packet(data), [(0, -33.5), (17, 1.0)])
        self.assertEqual(bridge.XPlaneReceiver.parse_rref_packet(b"DATA*xxxx"), [])

    def test_all_datarefs_fit(self):
        for _, path in bridge.DATAREFS:
            self.assertLess(len(path), 400)


class TestHelpers(unittest.TestCase):

    def test_track_from_ogl_velocity(self):
        self.assertAlmostEqual(bridge.track_from_velocity(0, -10), 0.0)    # north = -z
        self.assertAlmostEqual(bridge.track_from_velocity(10, 0), 90.0)
        self.assertAlmostEqual(bridge.track_from_velocity(0, 10), 180.0)
        self.assertAlmostEqual(bridge.track_from_velocity(-10, 0), 270.0)

    def test_vario_auto_falls_back_when_te_is_dead(self):
        selector = bridge.VarioSelector("auto")
        for t in range(5):
            selector.observe(float(t), 0.0, 2.0)
        self.assertEqual(selector.source, "raw")
        selector.observe(5.0, 1.5, 2.0)
        self.assertEqual(selector.source, "te")

    def test_vario_auto_keeps_te_in_level_flight(self):
        selector = bridge.VarioSelector("auto")
        for t in range(10):
            selector.observe(float(t), 0.0, 0.1)
        self.assertEqual(selector.source, "te")

    def test_rolling_mean_window(self):
        mean = bridge.RollingMean(1.0)
        mean.add(0.0, 10.0)
        mean.add(0.5, 2.0)
        mean.add(1.4, 4.0)
        self.assertAlmostEqual(mean.mean(), 3.0)


class TestEndToEnd(unittest.TestCase):
    """Bridge against the fake X-Plane over UDP loopback."""

    def setUp(self):
        self.fake = FakeXPlane(port=0, model=GliderModel(lat=-33.39, lon=-70.79, alt_m=1200.0,
                                                         climb_ms=1.8)).start()
        self.writer = ListWriter()
        self.app = bridge.XPlaneToSeeYou(xp_host="127.0.0.1", xp_port=self.fake.port,
                                          update_rate=4.0, rref_freq=20, debug_level=0,
                                          writer=self.writer)

    def tearDown(self):
        if self.app.running:
            self.app.stop()
        self.fake.stop()

    def test_flight_data_and_nmea(self):
        self.app.start_services()
        deadline = time.time() + 5
        while time.time() < deadline and len(self.writer.batches) < 4:
            time.sleep(0.05)
        self.assertGreaterEqual(len(self.writer.batches), 4, "no NMEA produced")

        fd = self.app.receiver.snapshot()
        self.assertAlmostEqual(fd.latitude, -33.39, delta=0.01)
        self.assertAlmostEqual(fd.longitude, -70.79, delta=0.01)
        self.assertAlmostEqual(fd.tas_ms, 25.0, delta=0.01)
        self.assertEqual(fd.vario_source, "TE")
        self.assertAlmostEqual(fd.vario_ms, 1.6, delta=0.7)
        self.assertAlmostEqual(fd.wind_dir_deg, 240.0, delta=0.01)
        self.assertFalse(fd.paused)

        gga, rmc, lx = self.writer.batches[-1]
        self.assertTrue(gga.startswith("$GPGGA,") and ",S,07047." in gga, gga)
        self.assertTrue(rmc.startswith("$GPRMC,") and ",A,3323." in rmc, rmc)
        fields = lx.split("*")[0].split(",")
        self.assertEqual(fields[:2], ["$LXWP0", "Y"])
        self.assertAlmostEqual(float(fields[2]), 90.0, delta=0.5)     # TAS 25 m/s
        self.assertEqual(fields[11], "240")
        self.assertAlmostEqual(float(fields[12]), 21.6, delta=0.1)    # 6 m/s wind
        for sentence in (gga, rmc, lx):
            body, checksum = sentence[1:].split("*")
            self.assertEqual(NMEA.checksum(body), checksum)

        self.app.stop()
        time.sleep(0.3)
        self.assertEqual(self.fake.subscriptions, {}, "bridge did not unsubscribe on stop")

    def test_no_output_without_xplane(self):
        self.fake.stop()
        self.app.start_services()
        time.sleep(1.0)
        self.assertEqual(self.writer.batches, [])
        self.fake = FakeXPlane(port=0)  # tearDown needs something to stop


if __name__ == "__main__":
    unittest.main()
