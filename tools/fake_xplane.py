#!/usr/bin/env python3
# Copyright (c) 2026 Juan Luis Gabriel
# This software is released under the MIT License.

"""
Fake X-Plane: answers RREF dataref subscriptions like X-Plane does, with a
glider circling in a thermal that drifts with the wind.

Use it to test the bridge (and SeeYou Navigator) without starting X-Plane:

    python tools/fake_xplane.py                 # listens on UDP 49000
    python xp12-seeyou-navigator-integration.py --dry-run

Only one program can listen on UDP 49000, so close X-Plane first, or run the
fake on another port and pass --xp-port to the bridge.
"""

import argparse
import math
import socket
import struct
import threading
import time
from typing import Dict, Tuple

EARTH_RADIUS_M = 6371000.0
FT_TO_M = 0.3048


class GliderModel:
    """Circling glider: constant bank, climbing, drifting downwind."""

    def __init__(self, lat=-33.3900, lon=-70.7900, alt_m=1200.0, tas_ms=25.0,
                 circle_s=22.0, climb_ms=1.8, wind_from_deg=240.0, wind_ms=6.0):
        self.lat0, self.lon0, self.alt0 = lat, lon, alt_m
        self.tas, self.circle_s, self.climb = tas_ms, circle_s, climb_ms
        self.wind_from, self.wind = wind_from_deg, wind_ms
        self.t0 = time.time()
        self.paused = False

    def state(self, now: float) -> Dict[str, float]:
        t = now - self.t0
        omega = 2 * math.pi / self.circle_s
        radius = self.tas / omega
        heading = math.degrees(omega * t) % 360.0
        # Air-relative position on the circle plus wind drift (wind blows TOWARD from+180)
        to_rad = math.radians(self.wind_from + 180.0)
        east = radius * math.sin(omega * t) + self.wind * math.sin(to_rad) * t
        north = -radius * math.cos(omega * t) + self.wind * math.cos(to_rad) * t
        hdg = math.radians(heading)
        v_east = self.tas * math.sin(hdg) + self.wind * math.sin(to_rad)
        v_north = self.tas * math.cos(hdg) + self.wind * math.cos(to_rad)
        lat = self.lat0 + math.degrees(north / EARTH_RADIUS_M)
        lon = self.lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(self.lat0))))
        alt = self.alt0 + self.climb * t
        vs = self.climb + 0.6 * math.sin(omega * t * 1.7)    # a bit of thermal texture
        zulu = (now % 86400.0)
        return {
            "sim/flightmodel/position/latitude": lat,
            "sim/flightmodel/position/longitude": lon,
            "sim/flightmodel/position/elevation": alt,
            "sim/flightmodel2/position/pressure_altitude": (alt - 90.0) / FT_TO_M,
            "sim/flightmodel/position/y_agl": alt - 480.0,
            "sim/flightmodel/position/groundspeed": math.hypot(v_east, v_north),
            "sim/flightmodel/position/local_vx": v_east,
            "sim/flightmodel/position/local_vz": -v_north,
            "sim/flightmodel/position/true_psi": heading,
            "sim/flightmodel/position/magnetic_variation": 1.5,
            "sim/flightmodel/position/indicated_airspeed": self.tas * 0.94 / 0.5144444,
            "sim/flightmodel/position/true_airspeed": self.tas,
            "sim/flightmodel/position/vh_ind": vs,
            "sim/cockpit2/gauges/indicators/total_energy_fpm": (vs - 0.2) / 0.00508,
            "sim/weather/aircraft/wind_now_direction_degt": self.wind_from,
            "sim/weather/aircraft/wind_now_speed_msc": self.wind,
            "sim/time/zulu_time_sec": zulu,
            "sim/time/paused": 1.0 if self.paused else 0.0,
        }


class FakeXPlane:
    """UDP server speaking the RREF part of the X-Plane protocol."""

    def __init__(self, port: int = 49000, model: GliderModel = None, host: str = "127.0.0.1"):
        self.model = model or GliderModel()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        # (client address, index) -> (freq, path)
        self.subscriptions: Dict[Tuple[Tuple[str, int], int], Tuple[int, str]] = {}
        self._last_sent: Dict[Tuple[str, int], float] = {}
        self.running = False
        self._thread = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(1.0)
        self.sock.close()

    def _handle_request(self, data: bytes, addr):
        if len(data) < 413 or data[:5] != b"RREF\x00":
            return
        freq, index = struct.unpack_from("<ii", data, 5)
        path = data[13:413].split(b"\x00", 1)[0].decode("ascii")
        key = (addr, index)
        if freq <= 0:
            self.subscriptions.pop(key, None)
        else:
            self.subscriptions[key] = (freq, path)

    def _loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                self._handle_request(data, addr)
                continue
            except socket.timeout:
                pass
            except OSError:
                pass
            now = time.time()
            clients = {}
            for (addr, index), (freq, path) in list(self.subscriptions.items()):
                clients.setdefault(addr, []).append((index, freq, path))
            for addr, subs in clients.items():
                freq = max(f for _, f, _ in subs)
                if now - self._last_sent.get(addr, 0.0) < 1.0 / freq:
                    continue
                self._last_sent[addr] = now
                state = self.model.state(now)
                payload = b"".join(struct.pack("<if", i, float(state.get(p, 0.0))) for i, _, p in subs)
                try:
                    self.sock.sendto(b"RREF," + payload, addr)
                except OSError:
                    pass


def main():
    parser = argparse.ArgumentParser(description="Fake X-Plane RREF server with a circling glider")
    parser.add_argument("--port", type=int, default=49000, help="UDP port (default: 49000)")
    parser.add_argument("--lat", type=float, default=-33.39)
    parser.add_argument("--lon", type=float, default=-70.79)
    parser.add_argument("--alt", type=float, default=1200.0, help="Start altitude, m MSL")
    parser.add_argument("--climb", type=float, default=1.8, help="Climb rate, m/s")
    args = parser.parse_args()

    fake = FakeXPlane(args.port, GliderModel(args.lat, args.lon, args.alt, climb_ms=args.climb))
    fake.start()
    print(f"Fake X-Plane listening on UDP {fake.port}. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(2)
            clients = {addr for addr, _ in fake.subscriptions}
            print(f"{len(fake.subscriptions)} datarefs subscribed by {len(clients)} client(s)", end="\r")
    except KeyboardInterrupt:
        fake.stop()


if __name__ == "__main__":
    main()
