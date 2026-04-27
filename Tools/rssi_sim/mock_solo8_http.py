#!/usr/bin/env python3
"""
Mock Solo8 /localrfstatus.json server for AntennaTracker RSSI tests.

Runs on a PC (e.g. Windows). It:
  - Listens to MAVLink over UDP and reads NAV_CONTROLLER_OUTPUT to learn
    the tracker pointing (bearing, pitch).
  - Simulates a moving aircraft "truth" direction (azimuth/elevation)
    using a simple kinematic model.
  - Serves /localrfstatus.json with LocalDemodStatus.sigLevA0/B0/sigValid,
    so ArduPilot AP_RSSI HTTP backend (RSSI_TYPE=7) can poll it.

This is intended for closed-loop testing of ModeRSSIScan without real
radios or servos.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def wrap_180(deg: float) -> float:
    x = (deg + 180.0) % 360.0 - 180.0
    return x


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.tracker_bearing_deg = 0.0
        self.tracker_pitch_deg = 0.0
        self.last_nav_ms = 0.0

        self.t0 = time.time()

        # latest computed RF status (what the HTTP handler will serve)
        self.sig_valid = True
        self.sig_a_dbm = -120.0
        self.sig_b_dbm = -120.0


def aircraft_truth(t: float, az_speed_dps: float, el_amplitude_deg: float, el_period_s: float) -> tuple[float, float]:
    """
    Returns (truth_az_deg, truth_el_deg).

    A simple model: aircraft azimuth spins at constant rate; elevation is a sine wave.
    """
    az = wrap_180(t * az_speed_dps)
    if el_period_s <= 0:
        el = 0.0
    else:
        el = el_amplitude_deg * math.sin(2.0 * math.pi * (t / el_period_s))
    return az, el


def dbm_from_pointing_error(
    err_deg: float,
    peak_dbm: float,
    beamwidth_deg: float,
    noise_db: float,
    floor_dbm: float,
) -> float:
    """
    Convert angular error to received power in dBm.

    Uses a Gaussian-ish pattern: dbm = peak - k*err^2, where k derives from beamwidth.
    """
    bw = max(1e-3, beamwidth_deg)
    k = 12.0 / (bw * bw)  # tuned so ~bw gives ~12dB drop
    dbm = peak_dbm - k * (err_deg * err_deg)
    dbm += random.gauss(0.0, noise_db)
    return max(floor_dbm, dbm)


def updater_thread(
    st: State,
    *,
    update_hz: float,
    az_speed_dps: float,
    el_amplitude_deg: float,
    el_period_s: float,
    peak_dbm: float,
    beamwidth_deg: float,
    noise_db: float,
    floor_dbm: float,
    chain_delta_db: float,
    valid_drop_rate: float,
    nav_timeout_s: float,
) -> None:
    period = 1.0 / max(1e-3, update_hz)
    while True:
        now = time.time()
        t = now - st.t0
        truth_az, truth_el = aircraft_truth(t, az_speed_dps, el_amplitude_deg, el_period_s)

        with st.lock:
            age_s = 1e9
            if st.last_nav_ms > 0:
                age_s = max(0.0, now - (st.last_nav_ms / 1000.0))
            have_nav = age_s <= nav_timeout_s
            bearing = st.tracker_bearing_deg
            pitch = st.tracker_pitch_deg

        # If we haven't received nav data recently, make sigValid false periodically
        sig_valid = have_nav and (random.random() >= valid_drop_rate)

        err_az = wrap_180(truth_az - bearing)
        err_el = truth_el - pitch
        # crude combined error metric:
        err = math.hypot(err_az, err_el)

        dbm = dbm_from_pointing_error(err, peak_dbm, beamwidth_deg, noise_db, floor_dbm)

        # two chains: let B be slightly worse by default
        dbm_a = dbm
        dbm_b = dbm - abs(chain_delta_db)

        with st.lock:
            st.sig_valid = sig_valid
            st.sig_a_dbm = dbm_a
            st.sig_b_dbm = dbm_b

        time.sleep(period)


class Handler(BaseHTTPRequestHandler):
    server_version = "mock-solo8-http/1.0"

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/localrfstatus.json":
            self.send_response(404)
            self.end_headers()
            return

        st: State = self.server.state  # type: ignore[attr-defined]
        with st.lock:
            body = {
                "LocalDemodStatus": {
                    "sigValid": bool(st.sig_valid),
                    "sigLevA0": float(round(st.sig_a_dbm, 2)),
                    "sigLevB0": float(round(st.sig_b_dbm, 2)),
                }
            }

        payload = json.dumps(body).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args) -> None:
        # keep stdout clean; enable if needed while debugging
        return


def mavlink_listener_thread(st: State, mavlink_in: str) -> None:
    # pymavlink is already used in ArduPilot autotest; on Windows install with:
    #   pip install pymavlink
    from pymavlink import mavutil  # type: ignore

    m = mavutil.mavlink_connection(mavlink_in, autoreconnect=True)
    while True:
        msg = m.recv_match(type="NAV_CONTROLLER_OUTPUT", blocking=True, timeout=1)
        if msg is None:
            continue
        # fields (common.xml): nav_bearing, nav_pitch
        # some dialects may not provide nav_pitch; fall back safely.
        nav_pitch = getattr(msg, "nav_pitch", None)
        if nav_pitch is None:
            # older/variant dialects: treat as 0 tilt
            nav_pitch = 0.0
        with st.lock:
            st.tracker_bearing_deg = float(msg.nav_bearing)
            st.tracker_pitch_deg = float(nav_pitch)
            st.last_nav_ms = time.time() * 1000.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0", help="HTTP listen address")
    ap.add_argument("--port", type=int, default=8080, help="HTTP listen port")
    ap.add_argument(
        "--mavlink-in",
        default="udp:0.0.0.0:14550",
        help='MAVLink input, e.g. "udp:0.0.0.0:14550" (listen) or "udp:127.0.0.1:14560"',
    )

    # aircraft truth model
    ap.add_argument("--az-speed-dps", type=float, default=10.0, help="truth azimuth rate (deg/s)")
    ap.add_argument("--el-amplitude-deg", type=float, default=8.0, help="truth elevation amplitude (deg)")
    ap.add_argument("--el-period-s", type=float, default=20.0, help="truth elevation period (s)")

    # RF model
    ap.add_argument("--peak-dbm", type=float, default=-45.0, help="peak received signal (dBm)")
    ap.add_argument("--beamwidth-deg", type=float, default=25.0, help="beamwidth proxy (deg)")
    ap.add_argument("--noise-db", type=float, default=1.5, help="noise sigma (dB)")
    ap.add_argument("--floor-dbm", type=float, default=-120.0, help="floor (dBm)")
    ap.add_argument("--chain-delta-db", type=float, default=2.0, help="B chain penalty (dB)")

    ap.add_argument("--update-hz", type=float, default=10.0, help="RF status update rate (Hz)")
    ap.add_argument("--valid-drop-rate", type=float, default=0.02, help="probability to report sigValid=false")
    ap.add_argument("--nav-timeout-s", type=float, default=2.0, help="if no NAV_CONTROLLER_OUTPUT for this long, sigValid becomes false")

    args = ap.parse_args()

    st = State()

    t_mav = threading.Thread(target=mavlink_listener_thread, args=(st, args.mavlink_in), daemon=True)
    t_mav.start()

    t_upd = threading.Thread(
        target=updater_thread,
        args=(st,),
        kwargs=dict(
            update_hz=args.update_hz,
            az_speed_dps=args.az_speed_dps,
            el_amplitude_deg=args.el_amplitude_deg,
            el_period_s=args.el_period_s,
            peak_dbm=args.peak_dbm,
            beamwidth_deg=args.beamwidth_deg,
            noise_db=args.noise_db,
            floor_dbm=args.floor_dbm,
            chain_delta_db=args.chain_delta_db,
            valid_drop_rate=args.valid_drop_rate,
            nav_timeout_s=args.nav_timeout_s,
        ),
        daemon=True,
    )
    t_upd.start()

    httpd = ThreadingHTTPServer((args.listen, args.port), Handler)
    httpd.state = st  # type: ignore[attr-defined]

    print(f"Mock Solo8 HTTP listening on http://{args.listen}:{args.port}/localrfstatus.json")
    print(f"MAVLink in: {args.mavlink_in} (listening for NAV_CONTROLLER_OUTPUT)")
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

