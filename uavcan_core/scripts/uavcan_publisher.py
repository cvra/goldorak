#!/usr/bin/env python
from uavcan_bridge.utils import load_uavcan_msg_dsdl, uavcan_publish
import uavcan
import argparse

def main():
    parser = argparse.ArgumentParser(description="Example node that publishes messages over UAVCAN.")
    parser.add_argument('--topic', '-t', default='cvra.motor.control.Velocity')
    parser.add_argument('--interface', '-i', default='vcan0')
    parser.add_argument('--id', default=10, type=int)
    parser.add_argument('--repeat', '-r', default=20, type=int)

    args, unknown = parser.parse_known_args()

    load_uavcan_msg_dsdl()
    node = uavcan.node.make_node(args.interface, node_id=args.id)

    for i in range(args.repeat):
        uavcan_publish(node, args.topic, {})
        node.spin(0.1)

if __name__ == '__main__':
    main()
