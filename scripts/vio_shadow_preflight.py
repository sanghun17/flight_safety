#!/usr/bin/env python3
"""Subscriber-only VIO/hybrid-IMU shadow recorder; creates no publishers."""

from __future__ import print_function

import inspect
import json
import math
import os
import socket
import threading
import time
from urllib.parse import urlparse
from xmlrpc.client import ServerProxy

import rospy
import rosgraph
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, UInt32

from flight_safety.append_only_evidence import (
    AppendOnlyRun, canonical_json_bytes, default_run_id, sha256_file, utc_now,
    validate_source_bundle_manifest)
from flight_safety.vio_shadow import (
    EVIDENCE_SOURCE_KEYS, HybridImuAccumulator, IdentityAccumulator,
    LinkageAccumulator, PRODUCER_IDENTITY_VALUE_KEYS, SCHEMA,
    StreamAccumulator, evaluate_candidate_provenance,
    normal_duration_completion, validate_profile)


MESSAGE_TYPES = {
    "geometry_msgs/PoseStamped": PoseStamped,
    "geometry_msgs/PoseWithCovarianceStamped": PoseWithCovarianceStamped,
    "nav_msgs/Odometry": Odometry,
    "sensor_msgs/Imu": Imu,
}
SCALAR_TYPES = {"std_msgs/String": String, "std_msgs/UInt32": UInt32}
EVIDENCE_EXTERNAL_SOURCE_LABELS = (
    EVIDENCE_SOURCE_KEYS - {"shadow_entrypoint", "bundle_manifest"})


def _stamp_ns(stamp):
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


def _now_ns():
    return _stamp_ns(rospy.Time.now())


def _vector3(value):
    return [value.x, value.y, value.z]


def _quaternion(value):
    return [value.x, value.y, value.z, value.w]


def _local_host_gate(host):
    if not isinstance(host, str) or not host:
        return False
    names = {"localhost", "127.0.0.1", "::1",
             socket.gethostname(), socket.getfqdn()}
    if host in names:
        return True
    try:
        remote = {item[4][0] for item in socket.getaddrinfo(host, None)}
        local = {"127.0.0.1", "::1"}
        for name in (socket.gethostname(), socket.getfqdn()):
            local.update(item[4][0] for item in socket.getaddrinfo(name, None))
    except socket.gaierror:
        return False
    return bool(remote & local)


class VioShadowNode(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.finished = False
        self.finish_reason = None
        self.started_at_utc = utc_now()
        self.started_monotonic = time.monotonic()
        self.started_wall_ns = time.monotonic_ns()
        self.profile = rospy.get_param("~profile")
        self.output_root = str(rospy.get_param("~output_root"))
        self.duration_s = float(rospy.get_param("~duration_s", 60.0))
        validate_profile(self.profile, self.duration_s, self.output_root)

        run_id = str(rospy.get_param("~run_id", "")) or default_run_id("vio-shadow")
        self.run = AppendOnlyRun(self.output_root, run_id)
        self.run_id = run_id
        profile_yaml = yaml.safe_dump(self.profile, default_flow_style=False, sort_keys=True)
        self.run.write_text("evidence/profile.yaml", profile_yaml, "application/x-yaml")
        self.source_identity = self._capture_source_identity()
        self.producer_provenance = self._capture_candidate_provenance()
        self.clock_publisher_runtime_cache = {}

        self.streams = {}
        self.identity = IdentityAccumulator(
            self.profile["identity"],
            self.producer_provenance["expected_typed_identity"])
        self.linkage = LinkageAccumulator(self.profile["linkage"])
        self.hybrid = HybridImuAccumulator(self.profile["hybrid_imu"])
        self.subscribers = []
        for stream_config in self.profile["streams"]:
            name = stream_config["name"]
            message_type_name = stream_config["message_type"]
            message_type = MESSAGE_TYPES[message_type_name]
            accumulator = StreamAccumulator(name, stream_config, self.started_wall_ns)
            self.streams[name] = accumulator
            self.subscribers.append(rospy.Subscriber(
                stream_config["topic"], message_type,
                self._stream_callback, callback_args=(name, message_type_name),
                queue_size=stream_config["queue_size"]))

        identity_config = self.profile["identity"]
        if identity_config["status_topic"]:
            self.subscribers.append(rospy.Subscriber(
                identity_config["status_topic"], DiagnosticArray,
                self._identity_callback, queue_size=1000))

        hybrid = self.profile["hybrid_imu"]
        self.subscribers.append(rospy.Subscriber(
            hybrid["ready_topic"], Bool, self._hybrid_ready_callback, queue_size=100))
        self.subscribers.append(rospy.Subscriber(
            hybrid["diagnostics_topic"], DiagnosticArray,
            self._hybrid_diagnostic_callback, queue_size=100))
        if hybrid["mapping_start_topic"]:
            self.subscribers.append(rospy.Subscriber(
                hybrid["mapping_start_topic"],
                SCALAR_TYPES[hybrid["mapping_start_message_type"]],
                self._mapping_start_callback, queue_size=100))
        if hybrid["timesync_domain_topic"]:
            self.subscribers.append(rospy.Subscriber(
                hybrid["timesync_domain_topic"], DiagnosticArray,
                self._timesync_domain_callback, queue_size=100))
        rospy.on_shutdown(self._shutdown_finish)

    def _capture_source_identity(self):
        evidence = self.profile["evidence"]
        configured = {}
        gates = []
        for label, path in sorted(evidence["source_paths"].items()):
            valid = (isinstance(path, str) and os.path.isabs(path) and
                     os.path.isfile(path) and not os.path.islink(path))
            artifact = None
            digest = None
            if valid:
                digest = sha256_file(path)
                with open(path, "rb") as stream:
                    artifact = self.run.write_bytes(
                        "evidence/sources/%s%s" % (label, os.path.splitext(path)[1]),
                        stream.read(), "application/octet-stream")
            configured[label] = {
                "path": path, "sha256": digest, "artifact": artifact, "gate": valid}
            gates.append(valid)
        actual_sources = {}
        for label, path in (
                ("collector_entrypoint_actual", os.path.realpath(__file__)),
                ("metric_core_actual", os.path.realpath(
                    inspect.getsourcefile(StreamAccumulator))),
                ("append_only_evidence_actual", os.path.realpath(
                    inspect.getsourcefile(AppendOnlyRun)))):
            valid = os.path.isfile(path)
            actual_sources[label] = {
                "path": path, "sha256": sha256_file(path) if valid else None,
                "gate": valid}
            gates.append(valid)
        configured_entrypoint = configured.get("shadow_entrypoint", {})
        actual_entrypoint = actual_sources["collector_entrypoint_actual"]
        configured_entrypoint["matches_actual_entrypoint"] = (
            configured_entrypoint.get("sha256") is not None and
            configured_entrypoint.get("sha256") == actual_entrypoint.get("sha256"))
        gates.append(configured_entrypoint["matches_actual_entrypoint"])
        manifest_paths = evidence["source_manifest_paths"]
        bundle_report = validate_source_bundle_manifest(
            evidence["source_paths"].get("bundle_manifest"),
            {
                "shadow_entrypoint": {
                    "path": manifest_paths["shadow_entrypoint"],
                    "sha256": configured["shadow_entrypoint"].get("sha256")},
                "metric_core": {
                    "path": "src/flight_safety/vio_shadow.py",
                    "sha256": actual_sources["metric_core_actual"].get("sha256")},
                "append_only_evidence": {
                    "path": "src/flight_safety/append_only_evidence.py",
                    "sha256": actual_sources["append_only_evidence_actual"].get("sha256")},
            },
            {
                label: {"path": manifest_paths[label],
                        "sha256": configured[label].get("sha256")}
                for label in sorted(EVIDENCE_EXTERNAL_SOURCE_LABELS)
            })
        gates.append(bundle_report["status"] == "PASS")
        messages = {}
        identity_message_types = dict(MESSAGE_TYPES)
        identity_message_types.update({
            "std_msgs/Bool": Bool, "std_msgs/String": String,
            "std_msgs/UInt32": UInt32,
            "diagnostic_msgs/DiagnosticArray": DiagnosticArray})
        for name, message_type in sorted(identity_message_types.items()):
            valid = (getattr(message_type, "_type", None) == name and
                     isinstance(getattr(message_type, "_md5sum", None), str) and
                     len(message_type._md5sum) == 32)
            messages[name] = {"runtime_type": getattr(message_type, "_type", None),
                              "runtime_md5": getattr(message_type, "_md5sum", None),
                              "gate": valid}
            gates.append(valid)

        def load_json_identity(label):
            path = configured.get(label, {}).get("path")
            if not isinstance(path, str) or not os.path.isfile(path):
                return None
            try:
                with open(path, "r") as stream:
                    value = json.load(stream)
                return value if isinstance(value, dict) else None
            except (OSError, ValueError):
                return None

        hybrid_build = load_json_identity("hybrid_imu_build_manifest")
        hybrid_build_gate = (
            isinstance(hybrid_build, dict) and set(hybrid_build) == {
                "schema", "hybrid_imu_node_sha256", "hybrid_imu_core_sha256",
                "hybrid_imu_launch_sha256"} and
            hybrid_build.get("schema") ==
                "flight_safety/hybrid_imu_runtime_build/v1" and
            hybrid_build.get("hybrid_imu_node_sha256") ==
                configured["hybrid_imu_node"].get("sha256") and
            hybrid_build.get("hybrid_imu_core_sha256") ==
                configured["hybrid_imu_core"].get("sha256") and
            hybrid_build.get("hybrid_imu_launch_sha256") ==
                configured["hybrid_imu_launch"].get("sha256"))
        clock_build = load_json_identity("clock_domain_publisher_build_manifest")
        clock_build_gate = (
            isinstance(clock_build, dict) and set(clock_build) == {
                "schema", "message_schema", "publisher_executable_sha256",
                "publisher_source_sha256", "hybrid_imu_build_manifest_sha256",
                "mavros_timesync_plugin_sha256"} and
            clock_build.get("schema") ==
                "flight_safety/d435_fcu_clock_domain_publisher_build/v1" and
            clock_build.get("message_schema") ==
                self.profile["hybrid_imu"]["timesync_domain_schema"] and
            clock_build.get("publisher_executable_sha256") ==
                configured["clock_domain_publisher_executable"].get("sha256") and
            clock_build.get("publisher_source_sha256") ==
                configured["clock_domain_publisher_source"].get("sha256") and
            clock_build.get("hybrid_imu_build_manifest_sha256") ==
                configured["hybrid_imu_build_manifest"].get("sha256") and
            clock_build.get("mavros_timesync_plugin_sha256") ==
                configured["mavros_timesync_plugin"].get("sha256"))
        gates.extend((hybrid_build_gate, clock_build_gate))
        required = evidence["require_source_identity"]
        return {
            "required": required, "configured_sources": configured,
            "actual_sources": actual_sources, "message_identities": messages,
            "bundle_manifest_gate": bundle_report,
            "hybrid_imu_build_join_gate": hybrid_build_gate,
            "clock_domain_publisher_build_join_gate": clock_build_gate,
            "status": "PASS" if (not required or all(gates)) else "FAIL",
            "source_to_binary_limitation": (
                "Native FAST-LIVO binary/library/source identity is independently required "
                "by producer_provenance; source hashes alone never satisfy that gate.")}

    def _capture_candidate_provenance(self):
        config = self.profile["producer_provenance"]
        configured = self.source_identity["configured_sources"]

        def load_yaml_source(label):
            path = configured.get(label, {}).get("path")
            if not isinstance(path, str) or not os.path.isfile(path):
                return None
            try:
                with open(path, "r") as stream:
                    value = yaml.safe_load(stream)
                return value if isinstance(value, dict) else None
            except (OSError, yaml.YAMLError):
                return None

        def load_json_source(label):
            path = configured.get(label, {}).get("path")
            if not isinstance(path, str) or not os.path.isfile(path):
                return None
            try:
                with open(path, "r") as stream:
                    value = json.load(stream)
                return value if isinstance(value, dict) else None
            except (OSError, ValueError):
                return None

        base = load_yaml_source("fastlivo_base_config")
        overlay = load_yaml_source("selected_candidate_overlay")
        camera = load_yaml_source("fastlivo_camera_config")
        build = load_json_source("fastlivo_build_manifest")
        namespace = config["runtime_param_namespace"]
        runtime_params = None
        if namespace:
            try:
                runtime_params = rospy.get_param(namespace)
            except (KeyError, rospy.ROSException):
                runtime_params = None
        camera_namespace = config["runtime_camera_param_namespace"]
        runtime_camera_params = None
        if camera_namespace:
            try:
                runtime_camera_params = rospy.get_param(camera_namespace)
            except (KeyError, rospy.ROSException):
                runtime_camera_params = None

        executable_path = config["executable_path"]
        executable_sha = None
        if (os.path.isabs(executable_path) and os.path.isfile(executable_path) and
                not os.path.islink(executable_path)):
            executable_sha = sha256_file(executable_path)
        library_sha = {}
        for name, path in sorted(config["dynamic_library_paths"].items()):
            if (os.path.isabs(path) and os.path.isfile(path) and
                    not os.path.islink(path)):
                library_sha[name] = sha256_file(path)

        source_sha = {}
        source_root = config["source_root"]
        build_files = (build.get("source_tree_identity", {}).get("files", {})
                       if isinstance(build, dict) else {})
        if (os.path.isabs(source_root) and os.path.isdir(source_root) and
                not os.path.islink(source_root) and isinstance(build_files, dict)):
            for relative in sorted(build_files):
                unresolved = os.path.join(source_root, relative)
                candidate = os.path.realpath(unresolved)
                inside = candidate.startswith(os.path.realpath(source_root) + os.sep)
                if (inside and os.path.isfile(candidate) and
                        not os.path.islink(unresolved)):
                    source_sha[relative] = sha256_file(candidate)
        report = evaluate_candidate_provenance(
            config, base, overlay, runtime_params, camera, runtime_camera_params,
            build, executable_sha, library_sha, source_sha)
        library_identity_fields = {
            "libimu_proc_sha256": library_sha.get("libimu_proc.so"),
            "liblaser_mapping_sha256": library_sha.get("liblaser_mapping.so"),
            "liblio_sha256": library_sha.get("liblio.so"),
            "libpre_sha256": library_sha.get("libpre.so"),
            "libvio_sha256": library_sha.get("libvio.so")}
        expected_typed_identity = {
            "selected_candidate_overlay_sha256": configured.get(
                "selected_candidate_overlay", {}).get("sha256"),
            "fastlivo_base_config_sha256": configured.get(
                "fastlivo_base_config", {}).get("sha256"),
            "fastlivo_camera_config_sha256": configured.get(
                "fastlivo_camera_config", {}).get("sha256"),
            "fastlivo_build_manifest_sha256": configured.get(
                "fastlivo_build_manifest", {}).get("sha256"),
            "effective_params_sha256": report.get("expected_effective_params_sha256"),
            "private_camera_params_sha256": report.get(
                "expected_static_camera_params_sha256"),
            "executable_sha256": executable_sha,
            "source_tree_sha256": (build.get("source_tree_sha256")
                                   if isinstance(build, dict) else None)}
        expected_typed_identity.update(library_identity_fields)
        report["expected_typed_identity"] = expected_typed_identity
        report["expected_typed_identity_schema_gate"] = (
            set(expected_typed_identity) == PRODUCER_IDENTITY_VALUE_KEYS and
            all(isinstance(value, str) and len(value) == 64
                for value in expected_typed_identity.values()))
        report["gates"]["typed_runtime_identity_recomputable"] = report[
            "expected_typed_identity_schema_gate"]
        report["status"] = (
            "PASS" if all(report["gates"].values()) else "FAIL")
        report["paths"] = {
            "base_config": configured.get("fastlivo_base_config", {}).get("path"),
            "selected_candidate_overlay": configured.get(
                "selected_candidate_overlay", {}).get("path"),
            "static_camera_config": configured.get(
                "fastlivo_camera_config", {}).get("path"),
            "build_manifest": configured.get("fastlivo_build_manifest", {}).get("path"),
            "executable": executable_path,
            "dynamic_libraries": dict(config["dynamic_library_paths"]),
            "source_root": source_root,
            "runtime_param_namespace": namespace,
            "runtime_camera_param_namespace": camera_namespace}
        return report

    def _stream_callback(self, message, callback_args):
        name, message_type_name = callback_args
        pose_covariance = None
        twist_covariance = None
        pose_position = None
        pose_orientation = None
        twist_linear = None
        twist_angular = None
        imu_angular_velocity = None
        imu_linear_acceleration = None
        child_frame_id = getattr(message, "child_frame_id", "")
        pose = None
        if message_type_name == "geometry_msgs/PoseStamped":
            pose = message.pose
        elif message_type_name == "geometry_msgs/PoseWithCovarianceStamped":
            pose = message.pose.pose
            pose_covariance = message.pose.covariance
        elif message_type_name == "nav_msgs/Odometry":
            pose = message.pose.pose
            pose_covariance = message.pose.covariance
            twist_covariance = message.twist.covariance
            twist_linear = _vector3(message.twist.twist.linear)
            twist_angular = _vector3(message.twist.twist.angular)
        elif message_type_name == "sensor_msgs/Imu":
            imu_angular_velocity = _vector3(message.angular_velocity)
            imu_linear_acceleration = _vector3(message.linear_acceleration)
            imu_orientation = _quaternion(message.orientation)
            imu_orientation_covariance = message.orientation_covariance
            imu_angular_velocity_covariance = message.angular_velocity_covariance
            imu_linear_acceleration_covariance = message.linear_acceleration_covariance
        else:
            imu_orientation = None
            imu_orientation_covariance = None
            imu_angular_velocity_covariance = None
            imu_linear_acceleration_covariance = None
        if message_type_name != "sensor_msgs/Imu":
            imu_orientation = None
            imu_orientation_covariance = None
            imu_angular_velocity_covariance = None
            imu_linear_acceleration_covariance = None
        if pose is not None:
            pose_position = _vector3(pose.position)
            pose_orientation = _quaternion(pose.orientation)
        with self.lock:
            if self.finished:
                return
            arrival_ros_ns = _now_ns()
            arrival_wall_ns = time.monotonic_ns()
            stamp_ns = _stamp_ns(message.header.stamp)
            self.streams[name].observe(
                stamp_ns, arrival_ros_ns, message.header.frame_id, child_frame_id,
                pose_covariance, twist_covariance, arrival_wall_ns,
                pose_position, pose_orientation, twist_linear, twist_angular,
                imu_angular_velocity, imu_linear_acceleration, imu_orientation,
                imu_orientation_covariance, imu_angular_velocity_covariance,
                imu_linear_acceleration_covariance)
            self.linkage.observe_stream(name, stamp_ns, arrival_wall_ns)

    def _identity_callback(self, message):
        arrival = time.monotonic_ns()
        arrival_ros = _now_ns()
        stamp = _stamp_ns(message.header.stamp)
        with self.lock:
            if self.finished:
                return
            for status in message.status:
                self.identity.observe_join(
                    status.name, status.level, status.message,
                    [(item.key, item.value) for item in status.values],
                    stamp, arrival, arrival_ros)

    def _hybrid_ready_callback(self, message):
        with self.lock:
            if not self.finished:
                self.hybrid.observe_ready(message.data, time.monotonic_ns())

    def _hybrid_diagnostic_callback(self, message):
        arrival = time.monotonic_ns()
        arrival_ros = _now_ns()
        stamp = _stamp_ns(message.header.stamp)
        with self.lock:
            if self.finished:
                return
            for status in message.status:
                self.hybrid.observe_diagnostic(
                    status.name, status.level, status.message,
                    [(item.key, item.value) for item in status.values], stamp, arrival,
                    arrival_ros)

    def _mapping_start_callback(self, message):
        with self.lock:
            if not self.finished:
                self.hybrid.observe_mapping_start(message.data, time.monotonic_ns())

    def _timesync_domain_callback(self, message):
        arrival = time.monotonic_ns()
        arrival_ros = _now_ns()
        stamp = _stamp_ns(message.header.stamp)
        connection_header = getattr(message, "_connection_header", {}) or {}
        callerid = connection_header.get("callerid")
        publisher_runtime = self._clock_publisher_runtime_identity(callerid)
        with self.lock:
            if self.finished:
                return
            for status in message.status:
                self.hybrid.observe_timesync_domain(
                    status.name, status.level, status.message,
                    [(item.key, item.value) for item in status.values],
                    stamp, arrival, arrival_ros, publisher_runtime)

    def _clock_publisher_runtime_identity(self, callerid):
        callerid = str(callerid or "")
        if callerid in self.clock_publisher_runtime_cache:
            return dict(self.clock_publisher_runtime_cache[callerid])
        result = {
            "callerid": callerid, "pid": None, "uri": None, "host": None,
            "local_host_gate": False, "executable_path": None,
            "executable_sha256": None}
        try:
            master = ServerProxy(rosgraph.get_master_uri())
            code, _message, uri = master.lookupNode(rospy.get_name(), callerid)
            if code != 1 or not isinstance(uri, str):
                raise ValueError("publisher node lookup failed")
            host = urlparse(uri).hostname
            local_gate = _local_host_gate(host)
            node = ServerProxy(uri)
            pid_code, _pid_message, pid = node.getPid(rospy.get_name())
            if pid_code != 1 or not isinstance(pid, int) or isinstance(pid, bool):
                raise ValueError("publisher PID lookup failed")
            executable = os.path.realpath("/proc/%d/exe" % pid)
            executable_sha = (sha256_file(executable)
                              if local_gate and os.path.isfile(executable) else None)
            result.update({
                "pid": pid, "uri": uri, "host": host,
                "local_host_gate": local_gate,
                "executable_path": executable,
                "executable_sha256": executable_sha})
        except (OSError, TypeError, ValueError, socket.error):
            pass
        self.clock_publisher_runtime_cache[callerid] = dict(result)
        return result

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if time.monotonic() - self.started_monotonic >= self.duration_s:
                self.finish("duration_complete")
                return
            rate.sleep()

    def _shutdown_finish(self):
        self.finish("ros_shutdown")

    def finish(self, reason="ros_shutdown"):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            self.finish_reason = str(reason)
            finished_monotonic = time.monotonic()
            now_wall_ns = time.monotonic_ns()
            now_ros_ns = _now_ns()
            actual_duration = finished_monotonic - self.started_monotonic
            stream_reports = {
                name: accumulator.summarize(
                    now_wall_ns, now_ros_ns, self.duration_s)
                for name, accumulator in sorted(self.streams.items())}
            identity_report = self.identity.summarize(now_wall_ns)
            linkage_report = self.linkage.summarize(self.identity.join_records)
            hybrid_output = stream_reports[self.profile["hybrid_imu"]["output_stream"]]
            expected_source_hashes = {
                label: record.get("sha256") for label, record in
                self.source_identity["configured_sources"].items()}
            hybrid_report = self.hybrid.summarize(
                now_wall_ns, hybrid_output, expected_source_hashes,
                identity_report["session_values"], identity_report["reset_values"])
            raw_records = [
                {"kind": "window_start", "arrival_wall_ns": self.started_wall_ns,
                 "started_at_utc": self.started_at_utc,
                 "duration_requested_s": self.duration_s}]
            for accumulator in self.streams.values():
                raw_records.extend(accumulator.raw_records)
            raw_records.extend(self.identity.raw_records)
            raw_records.extend(self.linkage.raw_records)
            raw_records.extend(self.hybrid.raw_records)
            raw_records.append({
                "kind": "window_finish", "arrival_wall_ns": now_wall_ns,
                "finish_reason": self.finish_reason, "duration_actual_wall_s": actual_duration})
        raw_records.sort(key=lambda item: (
            int(item.get("arrival_wall_ns", 0)), str(item.get("kind", "")),
            str(item.get("stream", ""))))
        raw_payload = b"".join(canonical_json_bytes(item) + b"\n" for item in raw_records)
        raw_artifact = self.run.write_bytes(
            "evidence/raw_observations.canonical.jsonl", raw_payload,
            "application/x-ndjson")
        duration_gate = normal_duration_completion(
            self.finish_reason, actual_duration, self.duration_s,
            self.profile["evidence"]["max_duration_overrun_s"])
        all_pass = (
            duration_gate and bool(stream_reports) and
            all(report["status"] == "PASS" for report in stream_reports.values()) and
            identity_report["status"] == "PASS" and
            linkage_report["status"] == "PASS" and
            hybrid_report["status"] == "PASS" and
            self.source_identity["status"] == "PASS" and
            self.producer_provenance["status"] == "PASS")
        report = {
            "schema": SCHEMA, "run_id": self.run_id,
            "started_at_utc": self.started_at_utc, "finished_at_utc": utc_now(),
            "duration_requested_s": self.duration_s,
            "duration_actual_wall_s": actual_duration,
            "finish_reason": self.finish_reason,
            "normal_duration_completion_gate": duration_gate,
            "collection_mode": "subscriber_only_shadow", "flight_authority": "NONE",
            "flight_readiness_interpretation": (
                "PASS validates only this shadow evidence contract; it never authorizes flight."),
            "safety_invariants": {
                "application_ros_publishers_created": False,
                "fcu_topics_written": False, "fcu_services_called": False,
                "parameters_set": False, "actuation_possible": False},
            "status": "PASS" if all_pass else "FAIL",
            "implementation_identity": self.source_identity,
            "producer_provenance": self.producer_provenance,
            "raw_evidence": {
                "artifact": raw_artifact, "record_count": len(raw_records),
                "encoding": "one canonical JSON object per line; NaN/Inf encoded as strings",
                "recompute_core_sha256": self.source_identity["actual_sources"][
                    "metric_core_actual"]["sha256"]},
            "streams": stream_reports, "identity": identity_report,
            "linkage": linkage_report, "hybrid_imu": hybrid_report,
            "limitations": {
                "mapping_start_binding": (
                    "No inferred binding is accepted; an explicit mapping-start event after "
                    "hybrid ready is required by the default profile."),
                "d435_fcu_clock_domain": (
                    "Header-stamp interpolation alone does not prove a shared D435/FCU clock; "
                    "an explicit fresh domain-status stream is required by the default profile."),
                "selected_candidate_phase_b_divergence": (
                    "Only the exact Phase-B online_intrinsics_en=false candidate is accepted. "
                    "A CameraInfo=true live profile is a separate unqualified shadow profile "
                    "requiring its own camera receipt and rebaseline; it must not be mixed here.")}}
        path, finalized = self.run.finalize(report)
        rospy.loginfo("VIO shadow receipt: %s status=%s sha256=%s",
                      path, finalized["status"], finalized["receipt_self_sha256"])


def main():
    rospy.init_node("vio_shadow_preflight", anonymous=False)
    node = VioShadowNode()
    node.spin()


if __name__ == "__main__":
    main()
