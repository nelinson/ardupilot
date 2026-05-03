# RSSI tracking simulator (Mock Solo8 HTTP)

This folder contains a small simulator to test AntennaTracker RSSI-based tracking without real radios or servos.

It is designed to work with the `AP_RSSI` Solo8 HTTP backend (`RSSI_TYPE=7`) by serving a minimal
`/localrfstatus.json` endpoint and generating dBm values based on the tracker pointing (pan/tilt).

## 1) Build / start AntennaTracker SITL

From the ArduPilot repo root:

```bash
Tools/autotest/sim_vehicle.py -v AntennaTracker -T --console
```

### Compile-time: enable the Solo8 RSSI HTTP backend in SITL

`RSSI_TYPE=7` is compiled only when `AP_SOLO8_RSSI_EXT_ENABLED=1` (see `libraries/AP_RSSI/AP_RSSI_config.h`).

Start SITL with:

```bash
Tools/autotest/sim_vehicle.py -v AntennaTracker -T --console \
  --configure-define AP_SOLO8_RSSI_EXT_ENABLED=1
```

## 2) Run the mock server on Windows

Install dependency:

```powershell
py -m pip install pymavlink
```

Run (example uses MAVProxy default out from WSL -> Windows: `udp:0.0.0.0:14550`):

```powershell
py Tools\rssi_sim\mock_solo8_http.py --port 8080 --mavlink-in "udp:0.0.0.0:14550"
```

## 3) Point AntennaTracker at the Windows server (MAVProxy console)

Assuming your Windows host IP from WSL2 is `172.22.112.1` (as seen in `sim_vehicle.py` output `--out 172.22.112.1:14550`):

```text
param set RSSI_TYPE 7
param set RSSI_HTTP_IP 172.22.112.1
param set RSSI_HTTP_PORT 8080
param set RSSI_HTTP_RATE 10
param set RSSI_HTTP_LOSS 1500
```

Then switch to your RSSI mode (mode number depends on your build; upstream-style mapping includes `RSSI_SCAN=6` and compass-gated `RSSI_SC=7` in `AntennaTracker/mode.h`). For bench scans with fixed PWM and AHRS grid sampling, use mode **7** (`INITIAL_MODE=7` after reboot, or select `RSSI_SC` in the GCS if exposed).

## Notes

- The mock server computes a moving "truth" aircraft direction and generates RSSI that peaks when tracker bearing/pitch matches it.
- Tracker pointing is taken from MAVLink `NAV_CONTROLLER_OUTPUT` (`nav_bearing` and `pitch`).

