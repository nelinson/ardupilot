Tools\rssi_sim\set_mode.py
@@ -0,0 +1,21 @@
# set_mode.py
import argparse
from pymavlink import mavutil

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", type=int, help="ArduPilot custom mode number (e.g. 6, 7)")
    ap.add_argument("--master", default="udp:0.0.0.0:14550", help='MAVLink connection string')
    ap.add_argument("--no-heartbeat-wait", action="store_true", help="send without waiting for heartbeat")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.master)

    if not args.no_heartbeat_wait:
        m.wait_heartbeat(timeout=10)

    m.set_mode_apm(args.mode)
    print(f"SET_MODE sent: {args.mode}")

if __name__ == "__main__":
    main()