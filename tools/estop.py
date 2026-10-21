#!/usr/bin/env python3
"""Emergency stop for every servo bus: arms, head and wheels.

Calls sts_hardware_interface's /emergency_stop_<component> service on each bus. The servos lose
torque, so the arms go limp and can fall, hold an arm before releasing a stopped robot. Each bus
service only covers its own bus, this calls all of them.

  python3 tools/estop.py            stop
  python3 tools/estop.py --release  release, joints hold where they are then, the controllers
                                    move them back toward their trajectory at the speed cap
"""

import argparse
import sys

import rclpy
from std_srvs.srv import SetBool

BUSES = ["xlerobot_bus1", "xlerobot_bus2"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--release", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("estop")
    clients = {b: node.create_client(SetBool, f"/emergency_stop_{b}") for b in BUSES}
    # send to every bus first, wait for the answers after, so the stop is not serial
    futures = {}
    for bus, client in clients.items():
        if client.wait_for_service(timeout_sec=5.0):
            futures[bus] = client.call_async(SetBool.Request(data=not args.release))
    ok = True
    for bus in BUSES:
        if bus not in futures:
            print(f"{bus}: service not available")
            ok = False
            continue
        rclpy.spin_until_future_complete(node, futures[bus], timeout_sec=5.0)
        result = futures[bus].result()
        print(f"{bus}: {result.message if result else 'no answer'}")
        ok = ok and bool(result and result.success)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
