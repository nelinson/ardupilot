from pymavlink import mavutil
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM7")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--mode", type=int, default=6)
args = ap.parse_args()

m = mavutil.mavlink_connection(args.port, baud=args.baud)
m.wait_heartbeat(timeout=10)

m.mav.set_mode_send(
    m.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    args.mode,
)

print(f"Sent SET_MODE custom_mode={args.mode} to sysid={m.target_system} via {args.port}@{args.baud}")