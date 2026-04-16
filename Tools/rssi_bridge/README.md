# rssi_bridge

Small Linux C++11 app that polls an HTTP JSON endpoint and injects RSSI into ArduPilot (AntennaTracker) by sending MAVLink `RADIO` messages over a USB serial connection.

## Build

Install prerequisites (Debian/Ubuntu/WSL2):

```bash
sudo apt update
sudo apt install -y build-essential cmake libcurl4-openssl-dev
```

Then build:

```bash
cd Tools/rssi_bridge
mkdir -p build
cd build
cmake ..
make -j
```

Notes:
- This project expects MAVLink C headers to exist (ArduPilot generates them into `build/*/libraries/GCS_MAVLink/include/`). Building ArduPilot once (SITL is fine) will generate them.
- If needed, point CMake at them explicitly:

```bash
cmake .. -DMAVLINK_INCLUDE_DIR=/path/to/ardupilot/build/sitl/libraries/GCS_MAVLink/include
```

## Windows build (native)

This project supports native Windows builds (HTTP via WinHTTP, serial via COM ports).

### Visual Studio + CMake

From a Developer PowerShell:

```powershell
cd Tools\rssi_bridge
cmake -S . -B build-win
cmake --build build-win --config Release
```

Run (replace COM port as needed):

```powershell
.\build-win\Release\rssi_bridge.exe --serial COM12
```

MAVLink headers:
- If ArduPilot-generated headers exist in this repo, CMake will use them.
- Otherwise CMake will fetch `mavlink/c_library_v2` automatically (can be disabled with `-DRSSI_BRIDGE_FETCH_MAVLINK=OFF`).

## Run

Default URL and serial device match the POC plan:

```bash
./rssi_bridge --serial /dev/ttyACM0
```

Dry-run (no serial I/O; just print computed RSSI):

```bash
./rssi_bridge --dry-run
```

Common fixes:
- Add your user to dialout:

```bash
sudo usermod -a -G dialout "$USER"
```

- If ModemManager grabs `/dev/ttyACM0`, stop it:

```bash
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
```

## ArduPilot / Mission Planner

For AntennaTracker to consume RSSI from MAVLink `RADIO` messages, set:
- `RSSI_TYPE = 5` (TelemetryRadioRSSI)

In Mission Planner you can verify reception:
- `Ctrl+F` -> MAVLink Inspector -> message `RADIO` -> field `rssi`

## Output

Each valid sample prints:

`[A:-100.0][B:-102.0][RSSI:73]`

If `sigValid` is false, the iteration is skipped.

