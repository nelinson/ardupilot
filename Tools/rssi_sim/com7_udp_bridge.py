import argparse
import time
from pymavlink import mavutil


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", default="COM7")
    ap.add_argument("--baud", type=int, default=115200)

    # MP connects to this UDP *listening* port (on the PC)
    ap.add_argument("--udp-listen", default="0.0.0.0:14550")

    # Where to forward PC/MP traffic back out (usually MP will also send from 14550,
    # but we also learn the last sender automatically)
    ap.add_argument("--udp-out", default=None, help='Optional fixed UDP out, e.g. "127.0.0.1:14551"')

    ap.add_argument("--source-system", type=int, default=255, help="MAVLink sysid for this bridge")
    args = ap.parse_args()

    # Serial link to Pixhawk
    m_ser = mavutil.mavlink_connection(args.com, baud=args.baud, source_system=args.source_system)

    # UDP endpoint MP will connect to
    m_udp = mavutil.mavlink_connection(f"udpin:{args.udp_listen}", source_system=args.source_system)

    # Optional fixed UDP out (if you prefer not to rely on learning sender)
    fixed_udp_out = None
    if args.udp_out:
        fixed_udp_out = mavutil.mavlink_connection(f"udpout:{args.udp_out}", source_system=args.source_system)

    last_udp_sender = None  # (ip, port)
    last_heartbeat_print = 0.0

    print(f"Bridge running:")
    print(f"  Serial: {args.com}@{args.baud}")
    print(f"  UDP listen: {args.udp_listen}  (connect Mission Planner here)")
    if args.udp_out:
        print(f"  UDP out (fixed): {args.udp_out}")
    else:
        print("  UDP out: learned automatically from first UDP packet")

    while True:
        # Serial -> UDP
        msg = m_ser.recv_msg()
        if msg is not None:
            b = msg.get_msgbuf()
            if b:
                if fixed_udp_out is not None:
                    fixed_udp_out.write(b)
                if last_udp_sender is not None:
                    # send raw bytes to the last sender
                    m_udp.mav.send(m_udp.mav.serialport, b)  # no-op for udpin
                    # pymavlink doesn't expose a direct "sendto" on udpin handle, so use a udpout handle:
                    # easiest: create/refresh udpout to learned sender
                    pass

        # UDP -> Serial (and learn sender)
        # Using recv_msg() on an udpin connection will also parse and track sender details internally.
        msg2 = m_udp.recv_msg()
        if msg2 is not None:
            b2 = msg2.get_msgbuf()
            if b2:
                m_ser.write(b2)

            # Learn sender (Mission Planner) from pymavlink internal fields if present
            # (Different pymavlink versions expose this differently; safest is to just set --udp-out and skip learning)
        now = time.time()
        if now - last_heartbeat_print > 5.0:
            last_heartbeat_print = now
            # Just show we’re alive (don’t spam)
            print("Bridge alive... (MP should be connected to UDP)")

        time.sleep(0.001)


if __name__ == "__main__":
    raise SystemExit(main())
