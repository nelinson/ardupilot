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

### Visual Studio 2026 (CMake presets)

This folder includes `CMakePresets.json` with a preset **`windows-vs2026-x64`** that uses the CMake generator **`Visual Studio 18 2026`**. That generator requires **CMake 4.2 or newer** (install the latest CMake from [cmake.org](https://cmake.org/download/) or use the version bundled with your Visual Studio install if it is new enough).

**Option A — Visual Studio IDE**

1. Install **Visual Studio 2026 Community** with workload **Desktop development with C++**.
2. Start Visual Studio → **Open a local folder** → select `Tools\rssi_bridge` (the folder that contains `CMakeLists.txt`).
3. When CMake configures, choose the configure preset **`windows-vs2026-x64`** (or from the menu: **Project → CMake presets for rssi_bridge → windows-vs2026-x64**).
4. Build configuration **Release**, then **Build → Build All**.

The executable is typically:

```text
Tools\rssi_bridge\build\windows-vs2026-x64\Release\rssi_bridge.exe
```

**Option B — If CMake does not list VS 2026 yet**

Use the **`windows-vs2022-x64`** preset instead (generator **Visual Studio 17 2022**), or from **Developer PowerShell for VS 2022**:

```powershell
cd Tools\rssi_bridge
cmake --preset windows-vs2022-x64
cmake --build --preset windows-vs2022-x64-release
```

### Visual Studio + CMake (command line, any generator)

From **Developer PowerShell** (so `cl.exe` is on PATH), you can still use a plain build directory:

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

