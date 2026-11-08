# xp12-seeyou-navigator-integration

Bridge that sends X-Plane 12 flight data to SeeYou Navigator via serial COM port (Naviter dongle).
Port of `../AFS4 Projects/afs4-seeyou-navigator-integration` (Aerofly FS4).

## Architecture

Single-file Python application (`xp12-seeyou-navigator-integration.py`), kept single-file so PyInstaller builds one exe:

| Class / function | Responsibility |
|---|---|
| `DATAREFS` | Ordered list of datarefs; the list index is the RREF index |
| `XPlaneReceiver` | RREF subscription over UDP (X-Plane port 49000), parses answers, keeps vario averages |
| `VarioSelector` | TE vario, raw vertical speed, or auto (TE, raw if TE stays at 0 while climbing/sinking) |
| `FlightData` | Immutable snapshot in SI units, built by `XPlaneReceiver.snapshot()` |
| `NMEAConverter` | GPGGA, GPRMC, LXWP0 in Condor 3's exact format |
| `SerialWriter` / `ConsoleWriter` | Output to COM port, or stdout for `--dry-run` |
| `XPlaneToSeeYou` | Orchestrator: snapshot → sentences → writer at 1 Hz; accepts an injected writer (tests) |
| `BridgeGUI` | Tkinter GUI (X-Plane IP, COM port, baud, live data) |

## X-Plane conventions (CRITICAL)

1. RREF values arrive as **float32**, even for double datarefs (lat/lon precision ~1 m, fine at 1 Hz NMEA).
2. RREF request = `b"RREF\0"` + `<ii400s` (freq, index, path) = 413 bytes. Answer = `b"RREF"` + 1 byte + N × `<if`. freq 0 unsubscribes.
3. X-Plane answers to the request's source port, so one socket on an ephemeral port does both.
4. On Windows, disable `SIO_UDP_CONNRESET` or `recvfrom()` raises when X-Plane is not running.
5. Local OGL velocity: **+x = east, +z = south**. Track = `atan2(vx, -vz)`.
6. `wind_now_direction_degt` is the direction the wind blows **FROM**, degrees true.
7. `total_energy_fpm` is ft/min; `vh_ind` is m/s; `pressure_altitude` is ft; `indicated_airspeed` is knots.

Dataref names were checked against `D:\Laminar\XP12-Last-Release\X-Plane 12\Resources\plugins\DataRefs.txt` (X-Plane 12.4.3).

## NMEA output

Same as the AFS4 project, verified byte for byte against the Condor 3 capture (`docs/condor3_capture.txt`, local only, git-ignored; the test `test_sentences_reproduce_condor3_capture` embeds its first cycle):

- **GPGGA** — 12 sats, HDOP 10, altitude MSL, trailing `0000`
- **GPRMC** — empty date / mag var / mode fields
- **LXWP0** — TAS kph, baro alt m, vario m/s, heading, wind dir, wind kph

Decisions:
- NMEA time defaults to **simulator zulu time** (Condor 3 sends sim time, see capture). `--time-source system` for the PC clock.
- LXWP0 altitude defaults to **true MSL** like Condor 3 (XCSoar's Condor driver reads it as true altitude; real LX devices send pressure altitude). `--baro-alt pressure` sends QNE.
- Nothing is sent while X-Plane is silent, so Navigator never sees a frozen fix.

## Testing

```bash
python -m unittest discover -v tests          # unit + end-to-end against fake X-Plane
python tools/fake_xplane.py                   # fake X-Plane on UDP 49000 (close X-Plane first)
python xp12-seeyou-navigator-integration.py --dry-run
```

## Key constraints

- Requires `pyserial` (not needed for `--dry-run`)
- Only one program can bind UDP 49000: the fake X-Plane and X-Plane cannot run together on the same port
