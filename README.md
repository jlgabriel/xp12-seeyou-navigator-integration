# X-Plane 12 SeeYou Navigator Integration

Bridge that connects [X-Plane 12](https://www.x-plane.com/) to [SeeYou Navigator](https://naviter.com/seeyou-navigator/) through a serial COM port and the [Naviter USB dongle](https://naviter.com/) (Silicon Labs CP210x).

The bridge reads the aircraft state from X-Plane, turns it into NMEA sentences and writes them to the dongle, the same way Condor 3 does. SeeYou Navigator on your phone treats X-Plane as if it were Condor.

Port of [afs4-seeyou-navigator-integration](https://github.com/jlgabriel/afs4-seeyou-navigator-integration) (Aerofly FS4) to X-Plane 12.

## Features

- Position, altitude, ground speed and track to SeeYou Navigator
- **Total energy vario** from X-Plane's TE gauge (falls back to raw vertical speed if the aircraft has none)
- **Real TAS and wind** straight from X-Plane, no estimation
- **Simulator time** in the NMEA sentences, like Condor 3
- Output identical in format to Condor 3 (verified against a capture, byte for byte)
- **Zero X-Plane setup**: no plugin, no Data Output settings; the bridge asks X-Plane for the datarefs it needs
- X-Plane can run on another PC of the local network
- Detects the Naviter dongle (CP210x) automatically and remembers the last settings
- GUI with live flight data, terminal mode, and a dry-run mode that prints the NMEA
- Fake X-Plane (`tools/fake_xplane.py`) to try everything without starting the simulator

## Requirements

- **Python 3.8+** and **pyserial** (`pip install pyserial`)
- **X-Plane 12** (X-Plane 11 also works: the bridge uses the classic UDP protocol)
- **Naviter USB dongle** connected to the PC, and **SeeYou Navigator** paired with it

## Data flow

```
X-Plane 12  (UDP 49000, RREF dataref subscription, 10 Hz)
     │
     │  position, altitude, speed, TE vario, TAS, wind, zulu time
     ▼
XP12 SeeYou Bridge
     ├─ $GPGGA  position + altitude MSL
     ├─ $GPRMC  position + ground speed + track
     └─ $LXWP0  TAS + baro altitude + vario + heading + wind
     │
Serial COM port (4800 baud, 1 Hz)  →  Naviter dongle  →  SeeYou Navigator
```

## Usage

```bash
pip install pyserial
python xp12-seeyou-navigator-integration.py
```

1. Start X-Plane 12 and load a glider (for example the Schleicher ASK 21).
2. Run the bridge. Check the COM port (the dongle is preselected if found) and click **Start**.
3. In SeeYou Navigator, connect to the dongle as you would with Condor.

### Terminal and dry-run modes

```bash
python xp12-seeyou-navigator-integration.py --no-gui --com-port COM7
python xp12-seeyou-navigator-integration.py --dry-run          # print NMEA, no COM port
python xp12-seeyou-navigator-integration.py --list-ports
```

### Options

```
--xp-host IP          IP of the X-Plane PC (default: 127.0.0.1)
--xp-port PORT        X-Plane UDP receive port (default: 49000)
--rref-freq HZ        Dataref rate requested from X-Plane (default: 10)
--com-port PORT       Serial port of the dongle (default: last used / auto-detected)
--baudrate RATE       4800, 9600, 19200, 38400 (default: 4800)
--update-rate HZ      NMEA output rate (default: 1, like Condor 3)
--baro-alt MODE       LXWP0 altitude: msl (default, like Condor 3) or pressure (QNE)
--time-source SRC     NMEA time: sim (default, X-Plane zulu time) or system (PC clock)
--vario MODE          auto (default), te (total energy) or raw (vertical speed)
--no-gui              Terminal mode
--dry-run             Print NMEA to the console instead of the COM port
--list-ports          List COM ports and exit
--debug LEVEL         0, 1, 2 (default: 1)
```

Settings chosen in the GUI (X-Plane IP, COM port, baud) are saved in `%APPDATA%\xp12-seeyou-bridge\settings.json`.

## Datarefs used

| Data | Dataref |
|---|---|
| Position | `sim/flightmodel/position/latitude`, `longitude`, `elevation` |
| Pressure altitude | `sim/flightmodel2/position/pressure_altitude` |
| Ground speed / track | `sim/flightmodel/position/groundspeed`, `local_vx`, `local_vz` |
| Heading | `sim/flightmodel/position/true_psi` |
| IAS / TAS | `sim/flightmodel/position/indicated_airspeed`, `true_airspeed` |
| Vario | `sim/cockpit2/gauges/indicators/total_energy_fpm`, `sim/flightmodel/position/vh_ind` |
| Wind | `sim/weather/aircraft/wind_now_direction_degt`, `wind_now_speed_msc` |
| Time / pause | `sim/time/zulu_time_sec`, `sim/time/paused` |

The vario sent to Navigator is the average of the last second (X-Plane sends 10 samples per second; Navigator gets one per second).

## Differences from the Aerofly FS4 version

| | AFS4 version | XP12 version |
|---|---|---|
| Data source | ForeFlight UDP broadcast + AeroflyReader DLL | RREF dataref subscription (no plugin) |
| Vario | DLL vertical speed or derived from altitude | X-Plane total energy vario |
| TAS | IAS converted with ISA model | X-Plane true airspeed |
| Wind | DLL only (ECEF → ENU) | X-Plane wind at the aircraft |
| NMEA time | PC clock | Simulator zulu time (like Condor 3) |
| X-Plane on another PC | — | Yes (`--xp-host`) |

## Troubleshooting

- **"X-Plane: Waiting..."**: X-Plane must be running with a flight loaded. If it runs on another PC, use its IP and allow UDP 49000 in that PC's firewall.
- **Serial: Disconnected**: check the port in Device Manager ("Silicon Labs CP210x USB to UART Bridge"). Close any other program using it (Condor, a sniffer).
- **Vario shows (RAW)**: the aircraft does not simulate a TE probe; the raw vertical speed is used instead. Force a source with `--vario te` or `--vario raw`.
- **Altitude in Navigator is off by the QNH difference**: try `--baro-alt pressure`.

## Tools

- `tools/fake_xplane.py` — fake X-Plane that answers RREF subscriptions with a glider circling in a thermal. Close X-Plane first (both use UDP 49000), or run it on another port with `--port` and pass the same `--xp-port` to the bridge.
- `tools/nmea_sniffer.py` — reads and logs NMEA from a COM port (for example through a com0com pair). It was used to capture Condor 3's output, which is the format reference for this bridge.

## Tests

```bash
python -m unittest discover -v tests
```

The tests include an end-to-end run against the fake X-Plane over UDP loopback, and a check that the bridge reproduces the Condor 3 capture exactly.

## Building the .exe

```bash
pip install pyinstaller
pyinstaller XP12-SeeYou-Bridge.spec
```

## License

MIT License — see [LICENSE](LICENSE).
