#!/usr/bin/env python3
"""
Minimal MAVLink text logger.

Captures STATUSTEXT (and a few optional message types) from a Pixhawk and writes
them to stdout and a log file.

Examples:
  # UDP (listen):
  python3 Tools/rssi_sim/mavlink_textlog.py --master "udp:0.0.0.0:14550"

  # Serial (USB):
  python3 Tools/rssi_sim/mavlink_textlog.py --master "COM7,115200"
  python3 Tools/rssi_sim/mavlink_textlog.py --master "/dev/ttyACM0,115200"
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import sys
import time


MAV_SEVERITY_STR = {
    0: "EMERGENCY",
    1: "ALERT",
    2: "CRITICAL",
    3: "ERROR",
    4: "WARNING",
    5: "NOTICE",
    6: "INFO",
    7: "DEBUG",
}


def severity_to_str(sev: object) -> str:
    if sev is None:
        return "UNKNOWN"
    try:
        sev_i = int(sev)
    except Exception:
        return f"UNKNOWN({sev})"
    return MAV_SEVERITY_STR.get(sev_i, f"UNKNOWN({sev_i})")


def utc_ts() -> str:
    # Use timezone-aware UTC timestamps (Python 3.12+ warns on utcnow()).
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def open_log(path: str | None) -> tuple[str, object]:
    if path is None:
        fn = f"mavlink_textlog_{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d_%H%M%S')}.log"
        path = os.path.abspath(fn)
    f = open(path, "a", encoding="utf-8")
    return path, f


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--master",
        required=True,
        help='MAVLink connection string: "udp:0.0.0.0:14550" or "COM7,115200" or "/dev/ttyACM0,115200"',
    )
    ap.add_argument("--log", default=None, help="Output log file path (default: auto timestamped in cwd)")
    ap.add_argument(
        "--types",
        default="STATUSTEXT",
        help='Comma-separated message types to log (default: "STATUSTEXT"). Example: "STATUSTEXT,NAV_CONTROLLER_OUTPUT"',
    )
    ap.add_argument("--no-heartbeat-wait", action="store_true", help="Do not wait for first heartbeat")
    ap.add_argument("--show-ids", action="store_true", help="Print sysid/compid for each line")
    args = ap.parse_args()

    try:
        from pymavlink import mavutil  # type: ignore
    except Exception as e:
        print(f"ERROR: pymavlink import failed: {e}", file=sys.stderr)
        print("Install with: python3 -m pip install pymavlink", file=sys.stderr)
        return 2

    types = [t.strip().upper() for t in args.types.split(",") if t.strip()]
    want_all = (len(types) == 0) or ("ALL" in types)
    want = set(types)

    log_path, log_f = open_log(args.log)
    print(f"Logging MAVLink from {args.master}")
    print(f"Types: {'ALL' if want_all else ','.join(sorted(want))}")
    print(f"Log file: {log_path}")

    m = mavutil.mavlink_connection(args.master, autoreconnect=True)

    if not args.no_heartbeat_wait:
        print("Waiting for heartbeat...")
        try:
            m.wait_heartbeat(timeout=30)
        except Exception:
            print("WARNING: heartbeat wait timed out (continuing).")

    last_print = 0.0
    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=1)
            if msg is None:
                now = time.time()
                if now - last_print > 5.0:
                    last_print = now
                    line = f"{utc_ts()} (no messages)"
                    print(line)
                    log_f.write(line + "\n")
                    log_f.flush()
                continue

            mtype = msg.get_type().upper()
            if not want_all and mtype not in want:
                continue

            prefix = utc_ts()
            if args.show_ids:
                prefix += f" sys={msg.get_srcSystem()} comp={msg.get_srcComponent()}"

            if mtype == "STATUSTEXT":
                # msg.text can be bytes or str depending on dialect/version
                txt = getattr(msg, "text", "")
                if isinstance(txt, (bytes, bytearray)):
                    txt = txt.decode(errors="replace")
                sev = getattr(msg, "severity", None)
                line = f"{prefix} {severity_to_str(sev)} {txt}".rstrip()
            else:
                line = f"{prefix} {mtype} {msg.to_dict()}"

            print(line)
            log_f.write(line + "\n")
            log_f.flush()
    except KeyboardInterrupt:
        print("\nExiting.")
        return 0
    finally:
        try:
            log_f.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

