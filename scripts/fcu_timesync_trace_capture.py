#!/usr/bin/env python3
"""Concurrent, subscriber-only MAVROS timesync qualification trace."""

from __future__ import print_function

import argparse
import json
import math
import sys
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import TimesyncStatus


SCHEMA = "flight_safety/fcu_timesync_trace/v1"
STREAM_TYPES = {
    "timesync_status": "mavros_msgs/TimesyncStatus",
    "diagnostics": "diagnostic_msgs/DiagnosticArray",
    "accepted_sequence": "diagnostic_msgs/DiagnosticArray",
}


def _header(header):
    return {
        "seq": int(header.seq),
        "stamp": {"secs": int(header.stamp.secs),
                  "nsecs": int(header.stamp.nsecs)},
        "frame_id": str(header.frame_id),
    }


def _diagnostic_payload(message):
    return {
        "header": _header(message.header),
        "status": [{
            "level": int(status.level), "name": str(status.name),
            "message": str(status.message),
            "hardware_id": str(status.hardware_id),
            "values": [{"key": str(item.key), "value": str(item.value)}
                       for item in status.values],
        } for status in message.status],
    }


def _timesync_payload(message):
    return {
        "header": _header(message.header),
        "remote_timestamp_ns": int(message.remote_timestamp_ns),
        "observed_offset_ns": int(message.observed_offset_ns),
        "estimated_offset_ns": int(message.estimated_offset_ns),
        "round_trip_time_ms": float(message.round_trip_time_ms),
    }


def _emit(record):
    print(json.dumps(record, sort_keys=True, separators=(",", ":"),
                     allow_nan=False), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Capture concurrent MAVROS timesync/diagnostic/acceptance evidence")
    parser.add_argument("--timesync-topic", required=True)
    parser.add_argument("--diagnostics-topic", required=True)
    parser.add_argument("--acceptance-topic", required=True)
    parser.add_argument("--timesync-min-samples", type=int, required=True)
    parser.add_argument("--diagnostic-min-samples", type=int, required=True)
    parser.add_argument("--acceptance-min-samples", type=int, required=True)
    parser.add_argument("--diagnostic-status-name", required=True)
    parser.add_argument("--acceptance-status-name", required=True)
    parser.add_argument("--connection-timeout", type=float, default=10.0)
    parser.add_argument("--capture-timeout", type=float, default=15.0)
    parser.add_argument("--tail-grace", type=float, default=0.5)
    args = parser.parse_args(rospy.myargv(
        argv=sys.argv if argv is None else argv)[1:])
    topics = {
        "timesync_status": args.timesync_topic,
        "diagnostics": args.diagnostics_topic,
        "accepted_sequence": args.acceptance_topic,
    }
    minimums = {
        "timesync_status": args.timesync_min_samples,
        "diagnostics": args.diagnostic_min_samples,
        "accepted_sequence": args.acceptance_min_samples,
    }
    if (any(not value.startswith("/") for value in topics.values()) or
            any(value < 1 for value in minimums.values()) or
            any(not math.isfinite(value) or value <= 0.0 for value in (
                args.connection_timeout, args.capture_timeout)) or
            not math.isfinite(args.tail_grace) or args.tail_grace < 0.0 or
            args.tail_grace >= args.capture_timeout):
        parser.error("invalid topic/sample/timeout contract")

    rospy.init_node("fcu_timesync_trace_capture", anonymous=True,
                    disable_signals=False)
    lock = threading.Lock()
    state = {"ready": False, "sequence": 0,
             "counts": {key: 0 for key in topics},
             "timesync_remotes": [], "acceptance_remotes": [],
             "eligible_after": None, "complete": False}

    def envelope(stream, payload):
        arrival_ros_ns = int(rospy.Time.now().to_nsec())
        arrival_steady_ns = int(time.monotonic_ns())
        sequence = state["sequence"]
        state["sequence"] += 1
        stamp_ns = int(payload["header"]["stamp"]["secs"] * 1000000000 +
                       payload["header"]["stamp"]["nsecs"])
        return {
            "schema": SCHEMA, "kind": "sample", "stream": stream,
            "topic": topics[stream], "message_type": STREAM_TYPES[stream],
            "collector_sequence": sequence,
            "arrival_ros_ns": arrival_ros_ns,
            "arrival_steady_ns": arrival_steady_ns,
            "header_stamp_ns": stamp_ns, "payload": payload,
        }

    def observe(stream, payload, qualifies=True):
        with lock:
            if not state["ready"]:
                return
            record = envelope(stream, payload)
            _emit(record)
            if qualifies:
                state["counts"][stream] += 1
            if stream == "timesync_status":
                state["timesync_remotes"].append(
                    payload.get("remote_timestamp_ns"))
            elif stream == "accepted_sequence" and qualifies:
                for status in payload.get("status", []):
                    if status.get("name") != args.acceptance_status_name:
                        continue
                    values = [item.get("value") for item in status.get("values", [])
                              if item.get("key") == "remote_timestamp_ns"]
                    if len(values) == 1 and values[0].isdigit():
                        state["acceptance_remotes"].append(int(values[0]))
            minimum_gate = all(
                state["counts"][key] >= minimums[key] for key in minimums)
            now = time.monotonic()
            if minimum_gate and state["eligible_after"] is None:
                state["eligible_after"] = now + args.tail_grace
            status_tail = state["timesync_remotes"][
                -args.acceptance_min_samples:]
            pair_gate = (
                len(status_tail) == args.acceptance_min_samples and
                len(set(status_tail)) == len(status_tail) and
                all(remote in set(state["acceptance_remotes"])
                    for remote in status_tail))
            if (minimum_gate and state["eligible_after"] is not None and
                    now >= state["eligible_after"] and pair_gate):
                # The callback holding this lock owns an exact paired suffix.
                # Freeze it before another callback can append a boundary row.
                state["ready"] = False
                state["complete"] = True

    def timesync_callback(message):
        observe("timesync_status", _timesync_payload(message))

    def diagnostics_callback(message):
        observe(
            "diagnostics", _diagnostic_payload(message),
            any(status.name == args.diagnostic_status_name
                for status in message.status))

    def acceptance_callback(message):
        observe(
            "accepted_sequence", _diagnostic_payload(message),
            any(status.name == args.acceptance_status_name
                for status in message.status))

    subscribers = {
        "timesync_status": rospy.Subscriber(
            topics["timesync_status"], TimesyncStatus, timesync_callback,
            queue_size=1000),
        "diagnostics": rospy.Subscriber(
            topics["diagnostics"], DiagnosticArray, diagnostics_callback,
            queue_size=1000),
        "accepted_sequence": rospy.Subscriber(
            topics["accepted_sequence"], DiagnosticArray, acceptance_callback,
            queue_size=1000),
    }
    connection_deadline = time.monotonic() + args.connection_timeout
    rate = rospy.Rate(100)
    while not rospy.is_shutdown() and any(
            subscriber.get_num_connections() < 1
            for subscriber in subscribers.values()):
        if time.monotonic() >= connection_deadline:
            print("timesync trace publisher connection handshake timed out",
                  file=sys.stderr)
            return 3
        rate.sleep()
    if rospy.is_shutdown():
        return 3
    with lock:
        ready = {
            "schema": SCHEMA, "kind": "ready",
            "collector_sequence": state["sequence"],
            "arrival_ros_ns": int(rospy.Time.now().to_nsec()),
            "arrival_steady_ns": int(time.monotonic_ns()),
            "streams": {
                key: {"topic": topics[key], "message_type": STREAM_TYPES[key],
                      "publisher_connections": int(
                          subscribers[key].get_num_connections())}
                for key in sorted(topics)},
        }
        state["sequence"] += 1
        _emit(ready)
        state["ready"] = True

    capture_deadline = time.monotonic() + args.capture_timeout
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock:
            if (not state["complete"] and state["eligible_after"] is not None and
                    now >= state["eligible_after"]):
                status_tail = state["timesync_remotes"][
                    -args.acceptance_min_samples:]
                pair_gate = (
                    len(status_tail) == args.acceptance_min_samples and
                    len(set(status_tail)) == len(status_tail) and
                    all(remote in set(state["acceptance_remotes"])
                        for remote in status_tail))
                if pair_gate:
                    state["ready"] = False
                    state["complete"] = True
            complete = state["complete"]
        if complete:
            return 0
        if now >= capture_deadline:
            print("timesync trace minimum sample contract timed out",
                  file=sys.stderr)
            return 3
        rate.sleep()
    return 3


if __name__ == "__main__":
    sys.exit(main())
