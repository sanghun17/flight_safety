#!/usr/bin/env python3
"""Subscriber-only FCU trace with collector-owned ROS and steady arrival time."""

from __future__ import print_function

import argparse
import json
import sys
import threading
import time

import rospy
from mavros_msgs.msg import ExtendedState, Param, State


SCHEMA = "flight_safety/fcu_trace_envelope/v1"
ROLE_TYPES = {
    "param": (Param, "mavros_msgs/Param"),
    "state": (State, "mavros_msgs/State"),
    "extended_state": (ExtendedState, "mavros_msgs/ExtendedState"),
}


def _header(header):
    return {
        "seq": int(header.seq),
        "stamp": {"secs": int(header.stamp.secs),
                  "nsecs": int(header.stamp.nsecs)},
        "frame_id": str(header.frame_id),
    }


def _payload(role, message):
    if role == "state":
        return {
            "header": _header(message.header),
            "connected": bool(message.connected), "armed": bool(message.armed),
            "guided": bool(message.guided),
            "manual_input": bool(message.manual_input),
            "mode": str(message.mode), "system_status": int(message.system_status),
        }
    if role == "extended_state":
        return {
            "header": _header(message.header),
            "vtol_state": int(message.vtol_state),
            "landed_state": int(message.landed_state),
        }
    return {
        "header": _header(message.header), "param_id": str(message.param_id),
        "value": {"integer": int(message.value.integer),
                  "real": float(message.value.real)},
        "param_index": int(message.param_index),
        "param_count": int(message.param_count),
    }


def _emit(record):
    print(json.dumps(record, sort_keys=True, separators=(",", ":"),
                     allow_nan=False), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Capture read-only MAVROS messages with actual arrival clocks")
    parser.add_argument("--role", choices=sorted(ROLE_TYPES), required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--count", type=int, default=0,
                        help="zero runs until terminated")
    parser.add_argument("--connection-timeout", type=float, default=10.0)
    args = parser.parse_args(rospy.myargv(argv=sys.argv if argv is None else argv)[1:])
    if (not args.topic.startswith("/") or args.count < 0 or
            args.connection_timeout <= 0.0):
        parser.error("invalid topic/count/connection timeout")

    message_class, message_type = ROLE_TYPES[args.role]
    rospy.init_node("fcu_trace_capture_%s" % args.role, anonymous=True,
                    disable_signals=False)
    lock = threading.Lock()
    state = {"ready": False, "samples": 0}

    def envelope(kind):
        return {
            "schema": SCHEMA, "kind": kind, "role": args.role,
            "topic": args.topic, "message_type": message_type,
            "arrival_ros_ns": int(rospy.Time.now().to_nsec()),
            "arrival_steady_ns": int(time.monotonic_ns()),
        }

    def callback(message):
        with lock:
            if not state["ready"]:
                return
            record = envelope("sample")
            record["header_stamp_ns"] = int(message.header.stamp.to_nsec())
            record["payload"] = _payload(args.role, message)
            _emit(record)
            state["samples"] += 1
            if args.count and state["samples"] >= args.count:
                rospy.signal_shutdown("requested sample count captured")

    subscriber = rospy.Subscriber(
        args.topic, message_class, callback, queue_size=1000)
    deadline = time.monotonic() + args.connection_timeout
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and subscriber.get_num_connections() < 1:
        if time.monotonic() >= deadline:
            print("publisher connection handshake timed out", file=sys.stderr)
            return 3
        rate.sleep()
    if rospy.is_shutdown():
        return 3
    with lock:
        ready = envelope("ready")
        ready["publisher_connections"] = int(subscriber.get_num_connections())
        _emit(ready)
        state["ready"] = True
    rospy.spin()
    return 0 if (args.count == 0 or state["samples"] == args.count) else 3


if __name__ == "__main__":
    sys.exit(main())
