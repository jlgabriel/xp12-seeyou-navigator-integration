# xp12-seeyou-navigator-integration

Bridge that sends X-Plane 12 flight data to SeeYou Navigator via serial COM port (Naviter dongle).
Port of `../AFS4 Projects/afs4-seeyou-navigator-integration` (Aerofly FS4).

## Architecture

Two front ends over one shared core package:

```
xp12_seeyou/                  shared package (no xp, no tkinter in core/serial_out)
  core.py                     DATAREFS, FlightData, VarioTracker, build_flight_data, NMEAConverter
  serial_out.py               SerialWriter, ThreadedSerialWriter, ConsoleWriter, dongle detection
  plugin.py                   XPPython3 PythonInterface (the only module importing XPPython3)
plugin/PI_xp12_seeyou_navigator.py   one-line XPPython3 entry, re-exports PythonInterface
xp12-seeyou-navigator-integration.py standalone app: RREF over UDP + Tkinter GUI (PyInstaller exe)
```

| Class / function | Responsibility |
|---|---|
| `DATAREFS` | Ordered (key, path, type) list; index = RREF index, type = getDatad/f/i in the plugin |
| `VarioTracker` | TE / raw selection (`VarioSelector`) plus 1 s and 30 s averages |
| `build_flight_data` | {key: value} map → immutable `FlightData` in SI units (both front ends use it) |
| `NMEAConverter` | GPGGA, GPRMC, LXWP0 in Condor 3's exact format |
| `ThreadedSerialWriter` | Serial output on its own thread; `submit()` never blocks the sim frame |
| `XPlaneReceiver` (standalone) | RREF subscription over UDP (X-Plane port 49000) |
| `PythonInterface` (plugin) | Flight loop at 10 Hz, NMEA at 1 Hz, Plugins menu, status window, settings |

## X-Plane conventions (CRITICAL)

1. RREF values arrive as **float32**, even for double datarefs (lat/lon ~1 m). The plugin reads doubles directly.
2. RREF request = `b"RREF\0"` + `<ii400s` (freq, index, path) = 413 bytes. Answer = `b"RREF"` + 1 byte + N × `<if`. freq 0 unsubscribes.
3. X-Plane answers to the request's source port, so one socket on an ephemeral port does both.
4. On Windows, disable `SIO_UDP_CONNRESET` or `recvfrom()` raises when X-Plane is not running.
5. Local OGL velocity: **+x = east, +z = south**. Track = `atan2(vx, -vz)`.
6. `wind_now_direction_degt` is the direction the wind blows **FROM**, degrees true.
7. `total_energy_fpm` is ft/min; `vh_ind` is m/s; `pressure_altitude` is ft; `indicated_airspeed` is knots.

Dataref names were checked against `D:\Laminar\XP12-Last-Release\X-Plane 12\Resources\plugins\DataRefs.txt` (X-Plane 12.4.3).

## Plugin rules

- Target **Python 3.12** (XPPython3's interpreter): run the tests with it too.
- Only the main (X-Plane) thread calls `xp.*`. The serial thread logs into a queue that the flight loop flushes.
- Never block the flight loop: serial open/write happen in `ThreadedSerialWriter`.
- XPPython3's "Reload scripts" does not reload submodules: reinstall with `tools/install_plugin.py` and restart X-Plane.
- pyserial must be installed in XPPython3's Python (Pip Package Installer, or its `python.exe -m pip install pyserial`).
- Settings: `<X-Plane>/Output/preferences/xp12_seeyou_navigator.json`. Log lines: `[SeeYou]` in `XPPython3Log.txt`.

## NMEA output

Verified byte for byte against the Condor 3 capture (`docs/condor3_capture.txt`, local only, git-ignored; the test `test_sentences_reproduce_condor3_capture` embeds its first cycle):

- **GPGGA** — 12 sats, HDOP 10, altitude MSL, trailing `0000`
- **GPRMC** — empty date / mag var / mode fields
- **LXWP0** — TAS kph, baro alt m, vario m/s, heading, wind dir, wind kph

Decisions:
- NMEA time defaults to **simulator zulu time** (Condor 3 sends sim time, see capture).
- LXWP0 altitude defaults to **true MSL** like Condor 3 (XCSoar's Condor driver reads it as true altitude; real LX devices send pressure altitude). Pressure altitude is an option.
- A missing TE dataref counts as a dead TE gauge, so `auto` vario falls back to raw.
- The standalone app sends nothing while X-Plane is silent, so Navigator never sees a frozen fix.

## Testing

```bash
python -m unittest discover -v tests          # standalone, core, plugin (fake xp module)
"D:/Laminar/XP12-Last-Release/X-Plane 12/Resources/plugins/XPPython3/win_x64/python.exe" -B -m unittest discover tests
python tools/install_plugin.py --xplane "D:/Laminar/XP12-Last-Release/X-Plane 12"   # X-Plane closed
python tools/fake_xplane.py                   # fake X-Plane on UDP 49000 (close X-Plane first)
python xp12-seeyou-navigator-integration.py --dry-run
```

Verified with hardware on 2026-11-08:
- Standalone app fed by the fake X-Plane reached SeeYou Navigator through the dongle on COM7.
- Plugin v1.1.0 in X-Plane 12.4.3 (XPPython3 4.7.1): menu and status window work, dongle auto-detected on COM7, data reached Navigator.

If Navigator shows nothing, restart the phone and check with Condor 3 first.

## Key constraints

- Requires `pyserial` (not needed for `--dry-run`)
- Plugin and standalone app must not run together: both open the same COM port
- Only one program can bind UDP 49000: the fake X-Plane and X-Plane cannot run together on the same port
