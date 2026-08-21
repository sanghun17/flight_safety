"""Fail-closed MAVROS/FCU read-only evidence collection.

The collector has no parameter-set, arm, mode, command, setpoint, or ROS
publisher path.  A forced parameter pull is persistent-FCU read-only, but it
does refresh MAVROS' ROS parameter cache; that bounded side effect is recorded
explicitly.  A receipt can PASS only when independently captured raw
``mavros_msgs/Param`` traffic proves that the dump is complete.
"""

from __future__ import print_function

import csv
import io
import json
import math
import os
import re
import socket
import subprocess
import tempfile
import time
from urllib.parse import urlparse

import yaml

from flight_safety.append_only_evidence import (
    AppendOnlyRun, default_run_id, sha256_file, utc_now,
    validate_source_bundle_manifest)


SCHEMA = "flight_safety/fcu_read_only_audit_receipt/v2"
TRACE_SCHEMA = "flight_safety/fcu_trace_envelope/v1"
TIMESYNC_TRACE_SCHEMA = "flight_safety/fcu_timesync_trace/v1"
TIMESYNC_ACCEPTANCE_SCHEMA = "flight_safety/mavros_timesync_acceptance/v1"
LANDED_STATE_ON_GROUND = 1
MAV_AUTOPILOT_PX4 = 12
PX4_HASH_PARAM = "_HASH_CHECK"
UINT16_MAX = 65535
_NAMESPACE_RE = re.compile(r"^/[A-Za-z][A-Za-z0-9_/]*$")
_PARAM_COUNT_RE = re.compile(r"Parameters received:\s*([0-9]+)")
_PID_RE = re.compile(r"(?:^|\n)Pid:\s*([1-9][0-9]*)(?:\n|$)")
_NODE_URI_RE = re.compile(
    r"(?:contacting node|URI:)\s+(https?://[^\s]+)", re.IGNORECASE)
_SERVICE_NODE_RE = re.compile(r"(?:^|\n)Node:\s*(/[^\s]+)(?:\n|$)")
_HEX_VERSION_RE = re.compile(r"^[0-9A-Fa-f]{8,64}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_UNSIGNED_TEXT_RE = re.compile(r"^(0|[1-9][0-9]*)$")
_SIGNED_TEXT_RE = re.compile(r"^(0|-?[1-9][0-9]*)$")
_FLOAT_TEXT_RE = re.compile(
    r"^-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$")

EXPECTED_MESSAGE_IDENTITIES = {
    "mavros_msgs/Param": "62165a8f212050223dda9583b0f22c3c",
    "mavros_msgs/TimesyncStatus": "021ec8044e747bea518b441f374ba64b",
    "mavros_msgs/EstimatorStatus": "39dbcc4be3f04b68422f784827c47dd5",
    "mavros_msgs/State": "65cd0a9fff993b062b91e354554ec7e9",
    "mavros_msgs/ExtendedState": "ae780b1800fe17b917369d21b90058bd",
    "mavros_msgs/VehicleInfo": "9afa55616f5936bd9469d7d85c523ac1",
    "diagnostic_msgs/DiagnosticArray": "60810da900de1dd6ddd437c3503511da",
}

ESTIMATOR_FIELDS = (
    "attitude_status_flag", "velocity_horiz_status_flag",
    "velocity_vert_status_flag", "pos_horiz_rel_status_flag",
    "pos_horiz_abs_status_flag", "pos_vert_abs_status_flag",
    "pos_vert_agl_status_flag", "const_pos_mode_status_flag",
    "pred_pos_horiz_rel_status_flag", "pred_pos_horiz_abs_status_flag",
    "gps_glitch_status_flag", "accel_error_status_flag")

PARAMETER_INVENTORY = (
    "SYS_AUTOSTART", "MAV_SYS_ID", "MAV_COMP_ID", "SYS_MC_EST_GROUP",
    "EKF2_EV_CTRL", "EKF2_AID_MASK", "EKF2_EV_DELAY", "EKF2_HGT_REF",
    "EKF2_EV_NOISE_MD", "EKF2_EVP_NOISE", "EKF2_EVV_NOISE",
    "EKF2_EVA_NOISE", "EKF2_GPS_CTRL", "EKF2_BARO_CTRL",
    "EKF2_RNG_CTRL", "EKF2_GPS_CHECK", "EKF2_REQ_EPH",
    "EKF2_REQ_EPV", "EKF2_REQ_NSATS", "EKF2_REQ_SACC", "EKF2_REQ_PDOP",
    "EKF2_REQ_HDRIFT", "EKF2_REQ_VDRIFT", "EKF2_REQ_GPS_H",
    "EKF2_REQ_GPS_V", "EKF2_NOAID_TOUT", "EKF2_TAU_POS", "EKF2_TAU_VEL",
    "COM_OF_LOSS_T", "COM_OBL_RC_ACT", "COM_RC_OVERRIDE",
    "COM_RC_STICK_OV", "COM_RCL_EXCEPT", "COM_POS_FS_DELAY",
    "COM_POS_FS_EPH", "COM_POS_FS_EPV", "COM_VEL_FS_EVH",
    "COM_POSCTL_NAVL", "COM_DISARM_LAND", "COM_DISARM_PRFLT",
    "COM_OBC_LOSS_T", "COM_OBC_LOSS_ACT",
    "NAV_DLL_ACT", "NAV_RCL_ACT", "RC_MAP_KILL_SW", "MPC_XY_VEL_MAX",
    "MPC_XY_CRUISE", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_JERK_MAX",
    "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_TILTMAX_AIR",
    "CBRK_VELPOSERR", "CBRK_FLIGHTTERM")

_SOURCE_PATH_LABELS = {
    "collector_entrypoint", "trace_entrypoint", "timesync_trace_entrypoint",
    "bundle_manifest", "mavparam_entrypoint", "mavparam_python_interpreter",
    "mavros_param_python", "mavros_param_runtime_module",
    "mavros_param_plugin", "mavros_param_plugin_binary",
    "mavros_param_source_tree_manifest", "mavros_param_build_manifest",
    "mavros_build_cmake_cache", "mavros_build_marker",
    "mavros_timesync_plugin", "mavros_timesync_config",
    "timesync_acceptance_publisher_source",
    "timesync_acceptance_publisher_binary",
    "timesync_acceptance_build_manifest",
}
_SOURCE_MANIFEST_LABELS = (_SOURCE_PATH_LABELS - {"bundle_manifest"}) | {
    "fcu_audit_config", "collector_core", "append_only_evidence"}
_TIMESYNC_EFFECTIVE_KEYS = {
    "conn/timesync_rate", "time/timesync_mode",
    "time/timesync_alpha_initial", "time/timesync_beta_initial",
    "time/timesync_alpha_final", "time/timesync_beta_final",
    "time/convergence_window", "time/max_rtt_sample",
    "time/max_deviation_sample", "time/max_consecutive_high_rtt",
    "time/max_consecutive_high_deviation", "time/publish_sim_time",
}


def _exact_keys(value, expected, label):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError("%s must have exact keys" % label)


def _validate_audit_config(config):
    """Reject policy weakening, typos, and non-finite qualification bounds."""
    public = {key: value for key, value in config.items()
              if not key.startswith("_")}
    _exact_keys(public, {
        "mavros_namespace", "command_timeout_s", "param_timeout_s",
        "min_param_count", "trace_startup_s", "trace_tail_s",
        "trace_connection_timeout_s", "state_guard", "identity",
        "timesync", "estimator_status"}, "FCU audit config")
    if (not isinstance(config.get("_config_source_path"), str) or
            not os.path.isabs(config["_config_source_path"])):
        raise ValueError("FCU audit config requires its actual absolute source path")
    for key in ("command_timeout_s", "param_timeout_s",
                "trace_connection_timeout_s"):
        if not _strict_number(public[key], 0.001):
            raise ValueError("%s must be finite and positive" % key)
    for key in ("trace_startup_s", "trace_tail_s"):
        if not _strict_number(public[key], 0.0):
            raise ValueError("%s must be finite and nonnegative" % key)
    if not _strict_int(public["min_param_count"], 1):
        raise ValueError("min_param_count must be a positive integer")

    state = public["state_guard"]
    _exact_keys(state, {
        "max_age_s", "max_future_s", "max_gap_s", "during_min_samples"},
                "state_guard")
    for key in ("max_age_s", "max_gap_s"):
        if not _strict_number(state[key], 0.001):
            raise ValueError("state_guard %s must be positive" % key)
    if (not _strict_number(state["max_future_s"], 0.0) or
            not _strict_int(state["during_min_samples"], 3)):
        raise ValueError("state_guard future/sample bounds are invalid")

    identity = public["identity"]
    _exact_keys(identity, {
        "required_autopilot", "require_vehicle_uid", "required_mavros_version",
        "source_paths", "source_manifest_paths"}, "identity")
    if (identity["required_autopilot"] != MAV_AUTOPILOT_PX4 or
            identity["require_vehicle_uid"] is not True or
            not isinstance(identity["required_mavros_version"], str) or
            not identity["required_mavros_version"]):
        raise ValueError("identity policy must require PX4, UID, and MAVROS version")
    _exact_keys(identity["source_paths"], _SOURCE_PATH_LABELS,
                "identity.source_paths")
    _exact_keys(identity["source_manifest_paths"], _SOURCE_MANIFEST_LABELS,
                "identity.source_manifest_paths")
    if (not all(isinstance(value, str)
                for value in identity["source_paths"].values()) or
            not all(isinstance(value, str) and value and not os.path.isabs(value)
                    for value in identity["source_manifest_paths"].values())):
        raise ValueError("identity paths have invalid types or logical paths")

    timesync = public["timesync"]
    _exact_keys(timesync, {
        "samples", "tail_samples", "diagnostic_samples", "diagnostic_name",
        "diagnostic_max_age_s", "diagnostic_max_gap_s", "max_age_s",
        "max_gap_s", "max_future_s", "max_abs_residual_ns",
        "acceptance_topic", "acceptance_samples", "acceptance_status_name",
        "acceptance_max_age_s", "acceptance_max_gap_s",
        "acceptance_link_max_delay_s", "acceptance_link_max_future_s",
        "connection_timeout_s", "capture_timeout_s", "tail_grace_s",
        "effective_config"}, "timesync")
    if (not _strict_int(timesync["samples"], 40) or
            not _strict_int(timesync["tail_samples"], 20) or
            timesync["samples"] <= timesync["tail_samples"] or
            not _strict_int(timesync["diagnostic_samples"], 2) or
            not _strict_int(timesync["acceptance_samples"],
                            timesync["tail_samples"])):
        raise ValueError("timesync qualification sample floors are invalid")
    for key in (
            "diagnostic_max_age_s", "diagnostic_max_gap_s", "max_age_s",
            "max_gap_s", "acceptance_max_age_s", "acceptance_max_gap_s",
            "acceptance_link_max_delay_s", "connection_timeout_s",
            "capture_timeout_s"):
        if not _strict_number(timesync[key], 0.001):
            raise ValueError("timesync %s must be positive" % key)
    for key in ("max_future_s", "acceptance_link_max_future_s", "tail_grace_s"):
        if not _strict_number(timesync[key], 0.0):
            raise ValueError("timesync %s must be nonnegative" % key)
    if (not _strict_int(timesync["max_abs_residual_ns"], 1) or
            not isinstance(timesync["diagnostic_name"], str) or
            not timesync["diagnostic_name"] or
            not isinstance(timesync["acceptance_status_name"], str) or
            not timesync["acceptance_status_name"] or
            not isinstance(timesync["acceptance_topic"], str) or
            not _NAMESPACE_RE.match(timesync["acceptance_topic"])):
        raise ValueError("timesync identity/residual contract is invalid")
    effective = timesync["effective_config"]
    _exact_keys(effective, _TIMESYNC_EFFECTIVE_KEYS,
                "timesync.effective_config")
    if (effective["time/publish_sim_time"] is not False or
            effective["time/timesync_mode"] != "MAVLINK" or
            not _strict_number(effective["conn/timesync_rate"], 0.001) or
            not _strict_int(effective["time/convergence_window"], 1)):
        raise ValueError("timesync effective clock-domain policy is invalid")

    estimator = public["estimator_status"]
    _exact_keys(estimator, {
        "samples", "max_age_s", "max_gap_s", "required_true_flags",
        "required_false_flags", "vertical_position_any_true"},
                "estimator_status")
    if (not _strict_int(estimator["samples"], 2) or
            not _strict_number(estimator["max_age_s"], 0.001) or
            not _strict_number(estimator["max_gap_s"], 0.001)):
        raise ValueError("estimator_status bounds are invalid")
    for key in ("required_true_flags", "required_false_flags",
                "vertical_position_any_true"):
        if (not isinstance(estimator[key], list) or not estimator[key] or
                len(estimator[key]) != len(set(estimator[key])) or
                not all(isinstance(item, str) for item in estimator[key])):
            raise ValueError("estimator_status flag lists must be exact/nonempty")


def load_config(path):
    with open(path, "r") as stream:
        value = yaml.safe_load(stream) or {}
    if not isinstance(value, dict):
        raise ValueError("audit config must be a mapping")
    value = dict(value)
    value["_config_source_path"] = os.path.abspath(path)
    return value


def _result(returncode=None, stdout="", stderr="", timed_out=False, error=None):
    return {
        "returncode": returncode, "stdout": stdout, "stderr": stderr,
        "timed_out": bool(timed_out), "error": error,
    }


def _default_runner(argv, timeout_s):
    try:
        completed = subprocess.run(
            argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=float(timeout_s), check=False)
        return _result(
            int(completed.returncode),
            completed.stdout.decode("utf-8", errors="replace"),
            completed.stderr.decode("utf-8", errors="replace"))
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or b""
        stderr = exc.stderr or b""
        if not isinstance(stdout, str):
            stdout = stdout.decode("utf-8", errors="replace")
        if not isinstance(stderr, str):
            stderr = stderr.decode("utf-8", errors="replace")
        return _result(
            None, stdout, stderr, True,
            "timeout after %.3fs" % float(timeout_s))
    except OSError as exc:
        return _result(
            None, "", "", False, "%s: %s" % (type(exc).__name__, exc))


def _yaml_documents(text):
    try:
        return [item for item in yaml.safe_load_all(text) if isinstance(item, dict)]
    except yaml.YAMLError:
        return []


_TRACE_MESSAGE_TYPES = {
    "param": "mavros_msgs/Param",
    "state": "mavros_msgs/State",
    "extended_state": "mavros_msgs/ExtendedState",
}

_TIMESYNC_TRACE_TYPES = {
    "timesync_status": "mavros_msgs/TimesyncStatus",
    "diagnostics": "diagnostic_msgs/DiagnosticArray",
    "accepted_sequence": "diagnostic_msgs/DiagnosticArray",
}


def _trace_envelopes(text, expected_role):
    """Parse collector-owned JSONL without trusting message header time as arrival."""
    failures = []
    records = []
    nonempty_lines = [line for line in str(text).splitlines() if line.strip()]
    for line_number, line in enumerate(nonempty_lines, 1):
        try:
            record = json.loads(line)
        except (TypeError, ValueError):
            failures.append("trace line %d is not JSON" % line_number)
            continue
        if not isinstance(record, dict):
            failures.append("trace line %d is not an object" % line_number)
            continue
        records.append(record)

    ready_records = []
    samples = []
    expected_type = _TRACE_MESSAGE_TYPES.get(expected_role)
    for index, record in enumerate(records):
        common_gate = (
            record.get("schema") == TRACE_SCHEMA and
            record.get("role") == expected_role and
            record.get("message_type") == expected_type and
            isinstance(record.get("topic"), str) and
            record.get("topic", "").startswith("/") and
            _strict_int(record.get("arrival_ros_ns"), 1) and
            _strict_int(record.get("arrival_steady_ns"), 1))
        if not common_gate:
            failures.append("trace record %d has invalid identity/arrival schema" % index)
            continue
        kind = record.get("kind")
        if kind == "ready":
            if set(record) != {
                    "schema", "kind", "role", "topic", "message_type",
                    "arrival_ros_ns", "arrival_steady_ns", "publisher_connections"}:
                failures.append("ready trace record has unknown/missing keys")
                continue
            if not _strict_int(record.get("publisher_connections"), 1):
                failures.append("trace ready record lacks a publisher connection")
                continue
            ready_records.append(record)
        elif kind == "sample":
            if set(record) != {
                    "schema", "kind", "role", "topic", "message_type",
                    "arrival_ros_ns", "arrival_steady_ns", "header_stamp_ns",
                    "payload"}:
                failures.append("sample trace record has unknown/missing keys")
                continue
            payload = record.get("payload")
            if (not isinstance(payload, dict) or
                    not _strict_int(record.get("header_stamp_ns"), 1) or
                    _header_stamp_ns(payload) != record.get("header_stamp_ns")):
                failures.append("trace sample payload/header schema is invalid")
                continue
            samples.append(record)
        else:
            failures.append("trace record kind is unknown")

    ready_gate = (
        len(ready_records) == 1 and bool(records) and
        records[0].get("kind") == "ready" and
        (not samples or
         ready_records[0]["arrival_steady_ns"] <= samples[0]["arrival_steady_ns"]))
    if not ready_gate:
        failures.append("trace lacks one leading connected ready handshake")
    topics = {record.get("topic") for record in records if isinstance(record, dict)}
    if len(topics) != 1:
        failures.append("trace records do not bind one exact topic")
    return {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "role": expected_role,
        "message_type": expected_type,
        "record_count": len(records),
        "sample_count": len(samples),
        "ready_handshake_gate": ready_gate,
        "ready_record": ready_records[0] if len(ready_records) == 1 else None,
        "samples": samples,
    }


def _trace_payloads(trace_report):
    return [record["payload"] for record in trace_report.get("samples", [])]


def _timesync_trace_envelopes(text, expected_topics, minimums, freshness):
    """Parse one collector-sequenced concurrent three-topic timesync window."""
    failures = []
    records = []
    for line_number, line in enumerate(
            line for line in str(text).splitlines() if line.strip()):
        try:
            record = json.loads(line)
        except (TypeError, ValueError):
            failures.append("timesync trace line %d is not JSON" % line_number)
            continue
        if not isinstance(record, dict):
            failures.append("timesync trace line %d is not an object" % line_number)
            continue
        records.append(record)
    ready = records[0] if records and records[0].get("kind") == "ready" else None
    ready_gate = (
        isinstance(ready, dict) and set(ready) == {
            "schema", "kind", "collector_sequence", "arrival_ros_ns",
            "arrival_steady_ns", "streams"} and
        ready.get("schema") == TIMESYNC_TRACE_SCHEMA and
        ready.get("collector_sequence") == 0 and
        _strict_int(ready.get("arrival_ros_ns"), 1) and
        _strict_int(ready.get("arrival_steady_ns"), 1) and
        isinstance(ready.get("streams"), dict) and
        set(ready["streams"]) == set(_TIMESYNC_TRACE_TYPES))
    if ready_gate:
        for stream, message_type in _TIMESYNC_TRACE_TYPES.items():
            stream_ready = ready["streams"].get(stream)
            ready_gate = ready_gate and (
                isinstance(stream_ready, dict) and set(stream_ready) == {
                    "topic", "message_type", "publisher_connections"} and
                stream_ready.get("topic") == expected_topics.get(stream) and
                stream_ready.get("message_type") == message_type and
                _strict_int(stream_ready.get("publisher_connections"), 1))
    if not ready_gate:
        failures.append(
            "timesync trace lacks one exact connected three-stream ready barrier")

    samples = {stream: [] for stream in _TIMESYNC_TRACE_TYPES}
    sequences = []
    arrivals_ros = []
    arrivals_steady = []
    for index, record in enumerate(records[1:], 1):
        stream = record.get("stream")
        exact_schema = (
            set(record) == {
                "schema", "kind", "stream", "topic", "message_type",
                "collector_sequence", "arrival_ros_ns", "arrival_steady_ns",
                "header_stamp_ns", "payload"} and
            record.get("schema") == TIMESYNC_TRACE_SCHEMA and
            record.get("kind") == "sample" and
            stream in _TIMESYNC_TRACE_TYPES and
            record.get("topic") == expected_topics.get(stream) and
            record.get("message_type") == _TIMESYNC_TRACE_TYPES.get(stream) and
            _strict_int(record.get("collector_sequence"), 1) and
            _strict_int(record.get("arrival_ros_ns"), 1) and
            _strict_int(record.get("arrival_steady_ns"), 1) and
            _strict_int(record.get("header_stamp_ns"), 1) and
            isinstance(record.get("payload"), dict) and
            _header_stamp_ns(record["payload"]) == record.get("header_stamp_ns"))
        if not exact_schema:
            failures.append(
                "timesync trace sample %d has invalid identity/arrival schema" % index)
            continue
        max_age_ns = int(float(freshness[stream]["max_age_s"]) * 1.0e9)
        max_future_ns = int(float(freshness[stream].get(
            "max_future_s", 0.05)) * 1.0e9)
        age = record["arrival_ros_ns"] - record["header_stamp_ns"]
        if not -max_future_ns <= age <= max_age_ns:
            failures.append(
                "timesync trace %s header is stale/future at actual arrival" % stream)
        samples[stream].append(record)
        sequences.append(record["collector_sequence"])
        arrivals_ros.append(record["arrival_ros_ns"])
        arrivals_steady.append(record["arrival_steady_ns"])

    sequence_gate = (
        bool(records) and len(sequences) == len(records) - 1 and
        sequences == list(range(1, len(records))))
    arrival_gate = (
        ready_gate and bool(arrivals_steady) and
        ready["arrival_ros_ns"] <= arrivals_ros[0] and
        ready["arrival_steady_ns"] <= arrivals_steady[0] and
        all(right >= left for left, right in zip(
            arrivals_ros, arrivals_ros[1:])) and
        all(right > left for left, right in zip(
            arrivals_steady, arrivals_steady[1:])))
    if not sequence_gate:
        failures.append("timesync trace collector sequence is not exact/contiguous")
    if not arrival_gate:
        failures.append("timesync trace collector arrivals are not monotonic")
    sample_count_gate = all(
        len(samples[stream]) >= int(minimums[stream]) for stream in samples)
    if not sample_count_gate:
        failures.append("timesync trace does not meet every stream sample minimum")
    concurrent_span_gate = (
        all(samples.values()) and
        max(items[0]["arrival_steady_ns"] for items in samples.values()) <=
        min(items[-1]["arrival_steady_ns"] for items in samples.values()))
    if not concurrent_span_gate:
        failures.append("timesync trace stream windows do not overlap")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "raw_record_count": len(records), "ready_barrier_gate": ready_gate,
        "collector_sequence_gate": sequence_gate,
        "collector_arrival_monotonic_gate": arrival_gate,
        "sample_minimum_gate": sample_count_gate,
        "concurrent_span_gate": concurrent_span_gate,
        "ready_record": ready, "samples": samples,
    }


def _strict_int(value, minimum=None, maximum=None):
    if not isinstance(value, int) or isinstance(value, bool):
        return False
    if minimum is not None and value < minimum:
        return False
    if maximum is not None and value > maximum:
        return False
    return True


def _strict_number(value, minimum=None, maximum=None):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    if not math.isfinite(float(value)):
        return False
    if minimum is not None and float(value) < float(minimum):
        return False
    if maximum is not None and float(value) > float(maximum):
        return False
    return True


def _header_stamp_ns(document):
    header = document.get("header")
    stamp = header.get("stamp") if isinstance(header, dict) else None
    if not isinstance(stamp, dict):
        return None
    secs = stamp.get("secs")
    nsecs = stamp.get("nsecs")
    if (not _strict_int(secs, 0) or
            not _strict_int(nsecs, 0, 999999999)):
        return None
    return secs * 1000000000 + nsecs


def _numeric_summary(documents, field):
    values = [document.get(field) for document in documents
              if _strict_number(document.get(field))]
    return {
        "count": len(values), "first": values[0] if values else None,
        "last": values[-1] if values else None,
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def _first_vehicle(documents):
    for document in documents:
        vehicles = document.get("vehicles")
        if isinstance(vehicles, list) and len(vehicles) == 1:
            if isinstance(vehicles[0], dict):
                return vehicles[0]
    return None


def _parse_numeric(text):
    value = yaml.safe_load(str(text))
    if not _strict_number(value):
        raise ValueError("parameter value is not a finite number")
    return value


def _parse_param_dump(text):
    parameters = {}
    errors = []
    for row_number, row in enumerate(csv.reader(io.StringIO(text)), 1):
        if not row or (row[0].strip() if row else "").startswith("#"):
            continue
        if len(row) != 2:
            errors.append("row %d has %d fields" % (row_number, len(row)))
            continue
        name = row[0].strip()
        if not name or name in parameters:
            errors.append("row %d has empty/duplicate name" % row_number)
            continue
        try:
            parameters[name] = _parse_numeric(row[1].strip())
        except (ValueError, yaml.YAMLError) as exc:
            errors.append("row %d: %s" % (row_number, exc))
    return parameters, errors


def _param_value(document):
    value = document.get("value")
    if not isinstance(value, dict):
        return None
    integer = value.get("integer")
    real = value.get("real")
    if not _strict_int(integer) or not _strict_number(real):
        return None
    if integer != 0:
        return integer
    if float(real) != 0.0:
        return float(real)
    return 0


def _numeric_equal(left, right):
    return (_strict_number(left) and _strict_number(right) and
            float(left) == float(right))


def _analyze_param_trace(samples, dump_parameters, reported_count,
                         autopilot, min_param_count, timing_config=None,
                         ready_record=None, dump_start_steady_ns=None,
                         dump_end_steady_ns=None):
    failures = []
    timing_config = timing_config or {}
    documents = [sample.get("payload") for sample in samples
                 if isinstance(sample, dict) and isinstance(sample.get("payload"), dict)]
    envelope_schema_gate = len(documents) == len(samples) and bool(samples)
    if not envelope_schema_gate:
        failures.append("raw parameter trace is not collector arrival envelopes")
    by_index = {}
    by_name = {}
    counts = set()
    exact_duplicates = 0
    pseudo = []
    malformed = 0
    stamps = []
    for document in documents:
        name = document.get("param_id")
        index = document.get("param_index")
        count = document.get("param_count")
        value = _param_value(document)
        stamp = _header_stamp_ns(document)
        if (not isinstance(name, str) or not name or
                not _strict_int(index, 0, UINT16_MAX) or
                not _strict_int(count, 1, UINT16_MAX) or value is None or
                stamp is None or stamp <= 0):
            malformed += 1
            continue
        stamps.append(stamp)
        record = (name, index, value, count)
        if index == UINT16_MAX:
            pseudo.append(record)
            continue
        counts.add(count)
        old_index = by_index.get(index)
        old_name = by_name.get(name)
        if old_index is not None:
            if old_index == record:
                exact_duplicates += 1
            else:
                failures.append("parameter index %d changed name/value/count" % index)
        if old_name is not None and old_name != record:
            failures.append("parameter %s changed index/value/count" % name)
        by_index[index] = record
        by_name[name] = record

    if malformed:
        failures.append("%d raw parameter messages have invalid schema" % malformed)
    if len(counts) != 1:
        failures.append("raw parameter messages do not advertise one stable count")
    advertised_count = next(iter(counts)) if len(counts) == 1 else None
    expected_indices = (set(range(advertised_count))
                        if advertised_count is not None else set())
    missing_indices = sorted(expected_indices - set(by_index))
    extra_indices = sorted(set(by_index) - expected_indices)
    if advertised_count is None or advertised_count < int(min_param_count):
        failures.append("advertised real parameter count is absent or too small")
    if missing_indices or extra_indices or len(by_index) != advertised_count:
        failures.append("raw parameter indices are not exactly 0..N-1")
    if len(by_name) != len(by_index):
        failures.append("raw parameter names are not unique")

    pseudo_count_semantics = (
        not pseudo or
        (advertised_count is not None and
         all(item[3] in (advertised_count, advertised_count + 1)
             for item in pseudo)))
    pseudo_allowed = (
        len(pseudo) <= 1 and
        all(item[0] == PX4_HASH_PARAM for item in pseudo) and
        autopilot == MAV_AUTOPILOT_PX4 and
        pseudo_count_semantics)
    if pseudo and not pseudo_allowed:
        failures.append(
            "index 65535 pseudo-parameter is not an evidenced PX4 _HASH_CHECK "
            "with advertised N/N+1 count semantics")
    if any(item[0] == PX4_HASH_PARAM for item in by_name.values()):
        failures.append("_HASH_CHECK appeared as a real indexed parameter")

    trace_names = set(by_name)
    dump_names = set(dump_parameters)
    if trace_names != dump_names:
        failures.append("parameter dump name set is not exact raw real-parameter set")
    mismatched_values = sorted(
        name for name in trace_names & dump_names
        if not _numeric_equal(by_name[name][2], dump_parameters[name]))
    if mismatched_values:
        failures.append("parameter dump values differ from raw trace")
    expected_reported = (
        advertised_count + (1 if pseudo else 0)
        if advertised_count is not None else None)
    if reported_count != expected_reported:
        failures.append("mavparam reported count does not match raw real+pseudo set")
    stamps_monotonic = all(
        right >= left for left, right in zip(stamps, stamps[1:]))
    if not stamps or not stamps_monotonic:
        failures.append("raw parameter message stamps are absent or non-monotonic")

    max_age_ns = int(float(timing_config.get("max_age_s", 1.5)) * 1.0e9)
    max_future_ns = int(float(timing_config.get("max_future_s", 0.05)) * 1.0e9)
    arrivals_ros = [sample.get("arrival_ros_ns") for sample in samples]
    arrivals_steady = [sample.get("arrival_steady_ns") for sample in samples]
    arrivals_valid = (
        bool(samples) and
        all(_strict_int(item, 1) for item in arrivals_ros + arrivals_steady))
    arrivals_monotonic = arrivals_valid and all(
        right > left for left, right in zip(arrivals_steady, arrivals_steady[1:]))
    arrival_ros_monotonic = arrivals_valid and all(
        right >= left for left, right in zip(arrivals_ros, arrivals_ros[1:]))
    header_arrival_fresh = (
        arrivals_valid and len(stamps) == len(samples) and
        all(-max_future_ns <= arrival - stamp <= max_age_ns
            for arrival, stamp in zip(arrivals_ros, stamps)))
    span_valid = (
        _strict_int(dump_start_steady_ns, 1) and
        _strict_int(dump_end_steady_ns, 1) and
        dump_start_steady_ns <= dump_end_steady_ns)
    ready_before_dump = (
        isinstance(ready_record, dict) and span_valid and
        _strict_int(ready_record.get("arrival_steady_ns"), 1) and
        ready_record["arrival_steady_ns"] <= dump_start_steady_ns)
    arrivals_inside_dump = (
        arrivals_valid and span_valid and
        all(dump_start_steady_ns <= stamp <= dump_end_steady_ns
            for stamp in arrivals_steady))
    if not arrivals_monotonic or not arrival_ros_monotonic:
        failures.append("raw parameter collector arrivals are absent/non-monotonic")
    if not header_arrival_fresh:
        failures.append("raw parameter headers are stale/future relative to collector arrival")
    if not ready_before_dump:
        failures.append("raw parameter subscriber was not ready before dump start")
    if not arrivals_inside_dump:
        failures.append("raw parameter arrivals were not inside the force-pull interval")

    return {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "raw_message_count": len(documents),
        "collector_envelope_gate": envelope_schema_gate,
        "malformed_message_count": malformed,
        "advertised_real_parameter_count": advertised_count,
        "unique_real_indices": len(by_index),
        "unique_real_names": len(by_name),
        "missing_indices": missing_indices,
        "extra_indices": extra_indices,
        "exact_retry_duplicate_count": exact_duplicates,
        "px4_hash_check_seen": bool(pseudo),
        "px4_hash_check_count_semantics_gate": pseudo_count_semantics,
        "px4_hash_check_exception_allowed": bool(pseudo and pseudo_allowed),
        "dump_name_set_exact": trace_names == dump_names,
        "dump_value_mismatch_names": mismatched_values,
        "reported_parameter_count": reported_count,
        "expected_reported_parameter_count": expected_reported,
        "stamps_monotonic": stamps_monotonic,
        "arrival_ros_stamps_monotonic": arrival_ros_monotonic,
        "arrival_steady_stamps_monotonic": arrivals_monotonic,
        "first_arrival_ros_ns": arrivals_ros[0] if arrivals_ros else None,
        "last_arrival_ros_ns": arrivals_ros[-1] if arrivals_ros else None,
        "first_arrival_steady_ns": arrivals_steady[0] if arrivals_steady else None,
        "last_arrival_steady_ns": arrivals_steady[-1] if arrivals_steady else None,
        "ready_arrival_steady_ns": (ready_record.get("arrival_steady_ns")
                                    if isinstance(ready_record, dict) else None),
        "header_fresh_at_actual_arrival_gate": header_arrival_fresh,
        "ready_before_dump_gate": ready_before_dump,
        "arrivals_inside_dump_gate": arrivals_inside_dump,
    }


def _phase_guard(state_samples, extended_samples, reference_ros_ns, config,
                 min_samples, state_ready=None, extended_ready=None,
                 span_start_steady_ns=None, span_end_steady_ns=None,
                 state_capture_start_steady_ns=None,
                 state_capture_end_steady_ns=None,
                 extended_capture_start_steady_ns=None,
                 extended_capture_end_steady_ns=None):
    max_age_ns = int(float(config.get("max_age_s", 1.5)) * 1.0e9)
    max_future_ns = int(float(config.get("max_future_s", 0.05)) * 1.0e9)
    max_gap_ns = int(float(config.get("max_gap_s", 2.0)) * 1.0e9)
    failures = []
    span_required = (
        span_start_steady_ns is not None or span_end_steady_ns is not None)
    span_valid = (
        not span_required or
        (_strict_int(span_start_steady_ns, 1) and
         _strict_int(span_end_steady_ns, 1) and
         span_end_steady_ns >= span_start_steady_ns))
    required_samples = int(min_samples)
    if span_valid and span_required:
        required_samples = max(
            required_samples,
            int(math.ceil(float(
                span_end_steady_ns - span_start_steady_ns) / max_gap_ns)) + 2)
    if span_required and not span_valid:
        failures.append("dump steady-time span is missing or invalid")
    def stamps_ok(samples, ready, label, capture_start, capture_end):
        capture_span_required = capture_start is not None or capture_end is not None
        capture_span_valid = (
            not capture_span_required or
            (_strict_int(capture_start, 1) and _strict_int(capture_end, 1) and
             capture_end >= capture_start))
        if capture_span_required and not capture_span_valid:
            failures.append("%s capture steady-time span is invalid" % label)
        headers = [item.get("header_stamp_ns") for item in samples]
        arrivals_ros = [item.get("arrival_ros_ns") for item in samples]
        arrivals_steady = [item.get("arrival_steady_ns") for item in samples]
        valid = (
            bool(samples) and
            all(_strict_int(item, 1) for item in
                headers + arrivals_ros + arrivals_steady))
        header_monotonic = valid and all(
            right > left for left, right in zip(headers, headers[1:]))
        arrival_ros_monotonic = valid and all(
            right >= left for left, right in zip(arrivals_ros, arrivals_ros[1:]))
        arrival_steady_monotonic = valid and all(
            right > left for left, right in zip(arrivals_steady, arrivals_steady[1:]))
        gap_ok = arrival_steady_monotonic and all(
            right - left <= max_gap_ns
            for left, right in zip(arrivals_steady, arrivals_steady[1:]))
        header_arrival_fresh = valid and all(
            -max_future_ns <= arrival - header <= max_age_ns
            for arrival, header in zip(arrivals_ros, headers))
        age = reference_ros_ns - arrivals_ros[-1] if valid else None
        fresh_at_finish = (
            age is not None and -max_future_ns <= age <= max_age_ns)
        ready_gate = (
            isinstance(ready, dict) and
            _strict_int(ready.get("arrival_steady_ns"), 1) and
            valid and ready["arrival_steady_ns"] <= arrivals_steady[0])
        covers_start = (
            span_valid and span_required and valid and
            arrivals_steady[0] <= span_start_steady_ns)
        covers_end = (
            span_valid and span_required and valid and
            arrivals_steady[-1] >= span_end_steady_ns)
        ready_before_span = (
            ready_gate and (not span_required or
                            ready["arrival_steady_ns"] <= span_start_steady_ns))
        within_capture = (
            not capture_span_required or
            (capture_span_valid and valid and isinstance(ready, dict) and
             capture_start <= ready.get("arrival_steady_ns", 0) <=
             arrivals_steady[0] and
             arrivals_steady[-1] <= capture_end))
        if len(samples) < required_samples:
            failures.append("%s sample count is below guard minimum" % label)
        if (not valid or not header_monotonic or
                not arrival_ros_monotonic or not arrival_steady_monotonic or
                not gap_ok or not header_arrival_fresh or not fresh_at_finish):
            failures.append(
                "%s actual arrivals/headers are not continuous and fresh" % label)
        if not ready_before_span:
            failures.append("%s ready handshake did not precede observation" % label)
        if not within_capture:
            failures.append(
                "%s actual arrivals are outside the command capture interval" % label)
        if span_required and (not covers_start or not covers_end):
            failures.append(
                "%s actual arrivals do not span dump start through finish" % label)
        return {
            "headers": headers, "arrivals_ros": arrivals_ros,
            "arrivals_steady": arrivals_steady, "age": age,
            "valid": valid, "header_monotonic": header_monotonic,
            "arrival_ros_monotonic": arrival_ros_monotonic,
            "arrival_steady_monotonic": arrival_steady_monotonic,
            "gap_gate": gap_ok,
            "header_arrival_fresh_gate": header_arrival_fresh,
            "fresh_at_finish_gate": fresh_at_finish,
            "ready_gate": ready_before_span,
            "within_capture_gate": within_capture,
            "capture_span_valid": capture_span_valid,
            "covers_start": covers_start, "covers_end": covers_end,
        }

    state_stamps = stamps_ok(
        state_samples, state_ready, "state", state_capture_start_steady_ns,
        state_capture_end_steady_ns)
    extended_stamps = stamps_ok(
        extended_samples, extended_ready, "extended_state",
        extended_capture_start_steady_ns, extended_capture_end_steady_ns)
    state_docs = _trace_payloads({"samples": state_samples})
    extended_docs = _trace_payloads({"samples": extended_samples})
    state_schema = all(
        isinstance(item.get("connected"), bool) and
        isinstance(item.get("armed"), bool) for item in state_docs)
    extended_schema = all(
        _strict_int(item.get("landed_state"), 0, 4) for item in extended_docs)
    if not state_schema:
        failures.append("state schema is incomplete")
    if not extended_schema:
        failures.append("extended_state schema is incomplete")
    connected = bool(state_docs) and state_schema and all(
        item["connected"] is True for item in state_docs)
    disarmed = bool(state_docs) and state_schema and all(
        item["armed"] is False for item in state_docs)
    on_ground = bool(extended_docs) and extended_schema and all(
        item["landed_state"] == LANDED_STATE_ON_GROUND for item in extended_docs)
    if not connected:
        failures.append("state guard did not remain connected")
    if not disarmed:
        failures.append("state guard did not remain explicitly disarmed")
    if not on_ground:
        failures.append("extended_state guard did not remain on-ground")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "state_samples": len(state_samples),
        "extended_state_samples": len(extended_samples),
        "required_samples_per_stream": required_samples,
        "dump_started_steady_ns": span_start_steady_ns,
        "dump_finished_steady_ns": span_end_steady_ns,
        "dump_span_timestamp_gate": span_valid,
        "state_capture_started_steady_ns": state_capture_start_steady_ns,
        "state_capture_finished_steady_ns": state_capture_end_steady_ns,
        "state_capture_span_timestamp_gate": state_stamps["capture_span_valid"],
        "extended_capture_started_steady_ns":
            extended_capture_start_steady_ns,
        "extended_capture_finished_steady_ns":
            extended_capture_end_steady_ns,
        "extended_capture_span_timestamp_gate": extended_stamps[
            "capture_span_valid"],
        "connected_all": connected, "disarmed_all": disarmed,
        "on_ground_all": on_ground,
        "state_first_stamp_ns": (state_stamps["headers"][0]
                                 if state_stamps["headers"] else None),
        "state_last_stamp_ns": (state_stamps["headers"][-1]
                                if state_stamps["headers"] else None),
        "extended_state_first_stamp_ns": (extended_stamps["headers"][0]
                                          if extended_stamps["headers"] else None),
        "extended_state_last_stamp_ns": (extended_stamps["headers"][-1]
                                         if extended_stamps["headers"] else None),
        "state_first_arrival_ros_ns": (state_stamps["arrivals_ros"][0]
                                       if state_stamps["arrivals_ros"] else None),
        "state_last_arrival_ros_ns": (state_stamps["arrivals_ros"][-1]
                                      if state_stamps["arrivals_ros"] else None),
        "state_first_arrival_steady_ns": (state_stamps["arrivals_steady"][0]
                                          if state_stamps["arrivals_steady"] else None),
        "state_last_arrival_steady_ns": (state_stamps["arrivals_steady"][-1]
                                         if state_stamps["arrivals_steady"] else None),
        "state_ready_arrival_steady_ns": (
            state_ready.get("arrival_steady_ns")
            if isinstance(state_ready, dict) else None),
        "extended_first_arrival_ros_ns": (extended_stamps["arrivals_ros"][0]
                                          if extended_stamps["arrivals_ros"] else None),
        "extended_last_arrival_ros_ns": (extended_stamps["arrivals_ros"][-1]
                                         if extended_stamps["arrivals_ros"] else None),
        "extended_first_arrival_steady_ns": (
            extended_stamps["arrivals_steady"][0]
            if extended_stamps["arrivals_steady"] else None),
        "extended_last_arrival_steady_ns": (
            extended_stamps["arrivals_steady"][-1]
            if extended_stamps["arrivals_steady"] else None),
        "extended_ready_arrival_steady_ns": (
            extended_ready.get("arrival_steady_ns")
            if isinstance(extended_ready, dict) else None),
        "state_covers_dump_start": state_stamps["covers_start"],
        "state_covers_dump_finish": state_stamps["covers_end"],
        "extended_state_covers_dump_start": extended_stamps["covers_start"],
        "extended_state_covers_dump_finish": extended_stamps["covers_end"],
        "state_last_age_s": (float(state_stamps["age"]) / 1.0e9
                             if state_stamps["age"] is not None else None),
        "extended_state_last_age_s": (float(extended_stamps["age"]) / 1.0e9
                                      if extended_stamps["age"] is not None else None),
        "state_stamps_valid": state_stamps["valid"],
        "state_stamps_monotonic": state_stamps["header_monotonic"],
        "state_arrival_ros_monotonic": state_stamps["arrival_ros_monotonic"],
        "state_arrival_steady_monotonic": state_stamps["arrival_steady_monotonic"],
        "state_gap_gate": state_stamps["gap_gate"],
        "state_fresh_gate": state_stamps["fresh_at_finish_gate"],
        "state_header_fresh_at_arrival_gate": state_stamps[
            "header_arrival_fresh_gate"],
        "state_ready_gate": state_stamps["ready_gate"],
        "state_arrivals_within_capture_gate": state_stamps["within_capture_gate"],
        "extended_stamps_valid": extended_stamps["valid"],
        "extended_stamps_monotonic": extended_stamps["header_monotonic"],
        "extended_arrival_ros_monotonic": extended_stamps[
            "arrival_ros_monotonic"],
        "extended_arrival_steady_monotonic": extended_stamps[
            "arrival_steady_monotonic"],
        "extended_gap_gate": extended_stamps["gap_gate"],
        "extended_fresh_gate": extended_stamps["fresh_at_finish_gate"],
        "extended_header_fresh_at_arrival_gate": extended_stamps[
            "header_arrival_fresh_gate"],
        "extended_ready_gate": extended_stamps["ready_gate"],
        "extended_arrivals_within_capture_gate": extended_stamps[
            "within_capture_gate"],
    }


def _measured_phase_failures(report, state_present, extended_present):
    """Return only violations proven by the stream rows that actually exist."""
    failures = []
    required = report.get("required_samples_per_stream")
    if state_present:
        if report.get("state_samples", 0) < required:
            failures.append("state measured sample count is below guard minimum")
        if not report.get("connected_all"):
            failures.append("state measured samples did not remain connected")
        if not report.get("disarmed_all"):
            failures.append("state measured samples did not remain explicitly disarmed")
        for key in (
                "state_stamps_valid", "state_stamps_monotonic",
                "state_arrival_ros_monotonic", "state_arrival_steady_monotonic",
                "state_gap_gate", "state_fresh_gate",
                "state_header_fresh_at_arrival_gate", "state_ready_gate",
                "state_arrivals_within_capture_gate"):
            if report.get(key) is False:
                failures.append("state measured timing gate failed: %s" % key)
    if extended_present:
        if report.get("extended_state_samples", 0) < required:
            failures.append(
                "extended_state measured sample count is below guard minimum")
        if not report.get("on_ground_all"):
            failures.append(
                "extended_state measured samples did not remain on-ground")
        for key in (
                "extended_stamps_valid", "extended_stamps_monotonic",
                "extended_arrival_ros_monotonic",
                "extended_arrival_steady_monotonic", "extended_gap_gate",
                "extended_fresh_gate", "extended_header_fresh_at_arrival_gate",
                "extended_ready_gate", "extended_arrivals_within_capture_gate"):
            if report.get(key) is False:
                failures.append(
                    "extended_state measured timing gate failed: %s" % key)
    return failures


def _estimator_report(documents, reference_ns, config):
    required_samples = int(config.get("samples", 10))
    max_gap_ns = int(float(config.get("max_gap_s", 1.0)) * 1.0e9)
    max_age_ns = int(float(config.get("max_age_s", 1.0)) * 1.0e9)
    true_flags = list(config.get("required_true_flags", []))
    false_flags = list(config.get("required_false_flags", []))
    vertical_any = list(config.get("vertical_position_any_true", []))
    failures = []
    known = set(ESTIMATOR_FIELDS)
    if (not set(true_flags).issubset(known) or
            not set(false_flags).issubset(known) or
            set(true_flags) & set(false_flags)):
        failures.append("estimator flag contract is invalid")
    vertical_known = {"pos_vert_abs_status_flag", "pos_vert_agl_status_flag"}
    if not vertical_any or not set(vertical_any).issubset(vertical_known):
        failures.append("vertical estimator gate contract is invalid")
    schema_ok = (
        len(documents) >= required_samples and
        all(all(isinstance(item.get(field), bool) for field in ESTIMATOR_FIELDS)
            for item in documents))
    if not schema_ok:
        failures.append("estimator_status continuous sample schema is incomplete")
    stamps = [_header_stamp_ns(item) for item in documents]
    stamp_valid = all(item is not None and item > 0 for item in stamps)
    monotonic = stamp_valid and all(
        right > left for left, right in zip(stamps, stamps[1:]))
    gaps_ok = monotonic and all(
        right - left <= max_gap_ns for left, right in zip(stamps, stamps[1:]))
    fresh = bool(stamps) and stamp_valid and 0 <= reference_ns - stamps[-1] <= max_age_ns
    if not stamp_valid or not monotonic or not gaps_ok or not fresh:
        failures.append("estimator_status samples are not continuous and fresh")
    true_gate = schema_ok and all(
        item.get(field) is True for item in documents for field in true_flags)
    false_gate = schema_ok and all(
        item.get(field) is False for item in documents for field in false_flags)
    vertical_gate = schema_ok and all(
        any(item.get(field) is True for field in vertical_any) for item in documents)
    if not true_gate:
        failures.append("required-true estimator flags were not continuously true")
    if not false_gate:
        failures.append("required-false estimator flags were not continuously false")
    if not vertical_gate:
        failures.append("vertical position estimator gate was not continuously satisfied")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "sample_count": len(documents), "required_samples": required_samples,
        "schema_valid": schema_ok, "stamps_monotonic": monotonic,
        "maximum_gap_gate": gaps_ok, "fresh_at_finish_gate": fresh,
        "required_true_flags": true_flags, "required_false_flags": false_flags,
        "vertical_position_any_true": vertical_any,
        "required_true_gate": true_gate, "required_false_gate": false_gate,
        "vertical_position_gate": vertical_gate,
        "last_sample": ({field: documents[-1].get(field)
                         for field in ESTIMATOR_FIELDS} if documents else None),
    }


def _parse_key_values(raw_values):
    normalized = {}
    duplicate_keys = []
    raw = []
    schema_valid = isinstance(raw_values, list)
    if not schema_valid:
        return normalized, duplicate_keys, raw, False
    for item in raw_values:
        if (not isinstance(item, dict) or set(item) != {"key", "value"} or
                not isinstance(item.get("key"), str) or not item.get("key") or
                not isinstance(item.get("value"), str)):
            schema_valid = False
            continue
        key = item["key"]
        raw.append({"key": key, "value": item["value"]})
        if key in normalized:
            duplicate_keys.append(key)
        else:
            normalized[key] = item["value"]
    return normalized, duplicate_keys, raw, schema_valid


def _diagnostic_records(documents, expected_name):
    failures = []
    matches = []
    total_statuses = 0
    raw_statuses = []
    for document_index, document in enumerate(documents):
        stamp = _header_stamp_ns(document)
        statuses = document.get("status") if isinstance(document, dict) else None
        if stamp is None or stamp <= 0 or not isinstance(statuses, list) or not statuses:
            failures.append("diagnostic document %d has invalid header/status array" %
                            document_index)
            continue
        for status_index, status in enumerate(statuses):
            total_statuses += 1
            status_map = status if isinstance(status, dict) else {}
            status_schema = (
                isinstance(status, dict) and
                set(status) == {"level", "name", "message", "hardware_id", "values"} and
                _strict_int(status.get("level"), 0, 255) and
                all(isinstance(status.get(key), str)
                    for key in ("name", "message", "hardware_id")))
            if not status_schema:
                failures.append(
                    "diagnostic status %d/%d has invalid schema" %
                    (document_index, status_index))
            values, duplicates, raw_values, values_valid = _parse_key_values(
                status_map.get("values"))
            raw_record = {
                "document_index": document_index,
                "status_index": status_index,
                "header_stamp_ns": stamp,
                "level": status_map.get("level"), "name": status_map.get("name"),
                "message": status_map.get("message"),
                "hardware_id": status_map.get("hardware_id"),
                "values_raw": raw_values,
                "duplicate_value_keys": sorted(set(duplicates)),
            }
            raw_statuses.append(raw_record)
            if not values_valid:
                failures.append(
                    "diagnostic status %d/%d has malformed KeyValue entries" %
                    (document_index, status_index))
            if duplicates:
                failures.append(
                    "diagnostic status %d/%d has duplicate KeyValue keys" %
                    (document_index, status_index))
            record = dict(raw_record)
            record["values"] = values
            record["status_schema_valid"] = status_schema
            if status_map.get("name") == expected_name:
                matches.append(record)
    if not documents or total_statuses == 0:
        failures.append("diagnostic evidence has no statuses")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "document_count": len(documents), "total_status_count": total_statuses,
        "matching": matches, "raw_statuses": raw_statuses,
    }


def _unsigned_text(value):
    if not isinstance(value, str) or not _UNSIGNED_TEXT_RE.match(value):
        return None
    return int(value)


def _signed_text(value):
    if not isinstance(value, str) or not _SIGNED_TEXT_RE.match(value):
        return None
    return int(value)


def _float_text(value):
    if not isinstance(value, str) or not _FLOAT_TEXT_RE.match(value):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _diagnostic_status(documents, expected_name, reference_ns,
                       max_age_s, max_gap_s):
    parsed = _diagnostic_records(documents, expected_name)
    matches = parsed["matching"]
    levels = [record["level"] for record in matches]
    stamps = [record["header_stamp_ns"] for record in matches]
    all_levels_ok = bool(levels) and all(level == 0 for level in levels)
    counters = [
        _unsigned_text(record["values"].get("Timesyncs since startup"))
        for record in matches]
    counters_valid = bool(counters) and all(item is not None for item in counters)
    counter_no_rollback = counters_valid and all(
        right >= left for left, right in zip(counters, counters[1:]))
    stamps_monotonic = bool(stamps) and all(
        right > left for left, right in zip(stamps, stamps[1:]))
    gaps_ok = stamps_monotonic and all(
        right - left <= float(max_gap_s) * 1.0e9
        for left, right in zip(stamps, stamps[1:]))
    fresh = (bool(stamps) and
             0 <= reference_ns - stamps[-1] <= float(max_age_s) * 1.0e9)
    ok = (parsed["status"] == "PASS" and all_levels_ok and
          counters_valid and counter_no_rollback and
          stamps_monotonic and gaps_ok and fresh)
    return {
        "matching_samples": len(matches), "levels": levels,
        "all_matching_levels_ok": all_levels_ok,
        "timesyncs_since_startup_sequence": counters,
        "counter_schema_valid": counters_valid,
        "counter_no_rollback_gate": counter_no_rollback,
        "observation_counter_is_not_acceptance_evidence": True,
        "header_stamps_monotonic": stamps_monotonic,
        "maximum_gap_gate": gaps_ok, "fresh_at_finish_gate": fresh,
        "all_diagnostic_status_schema_gate": parsed["status"] == "PASS",
        "diagnostic_parse_failures": parsed["failures"],
        "raw_statuses": parsed["raw_statuses"], "ok": ok,
    }


_ACCEPTANCE_KEYS = {
    "schema", "source_plugin_sha256", "publisher_source_sha256",
    "publisher_binary_sha256", "publisher_build_manifest_sha256",
    "session_id", "reset_counter", "observation_sequence",
    "accepted_sequence", "accepted", "converged", "remote_timestamp_ns",
    "round_trip_time_ms", "deviation_ns",
}


def _timesync_acceptance_report(documents, reference_ns, config, effective,
                                source_hashes, timesync_samples,
                                acceptance_samples):
    name = str(config.get(
        "acceptance_status_name", "flight_safety: MAVROS Time Sync Acceptance"))
    parsed = _diagnostic_records(documents, name)
    records = parsed["matching"]
    levels = [record["level"] for record in records]
    all_levels_ok = bool(levels) and all(level == 0 for level in levels)
    required = int(config.get("tail_samples", 20))
    max_age_ns = int(float(config.get("acceptance_max_age_s", 1.0)) * 1.0e9)
    max_gap_ns = int(float(config.get("acceptance_max_gap_s", 0.5)) * 1.0e9)
    failures = list(parsed["failures"])
    if not all_levels_ok:
        failures.append(
            "typed accepted-sequence diagnostic window is not entirely OK")
    typed_all = []
    for index, record in enumerate(records):
        values = record["values"]
        if set(values) != _ACCEPTANCE_KEYS:
            failures.append("accepted-sequence record %d has non-exact keys" % index)
            continue
        reset_counter = _unsigned_text(values.get("reset_counter"))
        observation_sequence = _unsigned_text(values.get("observation_sequence"))
        accepted_sequence = _unsigned_text(values.get("accepted_sequence"))
        remote_timestamp_ns = _unsigned_text(values.get("remote_timestamp_ns"))
        rtt = _float_text(values.get("round_trip_time_ms"))
        deviation = _signed_text(values.get("deviation_ns"))
        schema_gate = (
            values.get("schema") == TIMESYNC_ACCEPTANCE_SCHEMA and
            isinstance(values.get("session_id"), str) and
            bool(values.get("session_id")) and
            reset_counter is not None and observation_sequence is not None and
            accepted_sequence is not None and remote_timestamp_ns is not None and
            rtt is not None and deviation is not None and
            values.get("accepted") in ("true", "false") and
            values.get("converged") in ("true", "false") and
            all(_SHA256_RE.match(values.get(key, "")) is not None for key in (
                "source_plugin_sha256", "publisher_source_sha256",
                "publisher_binary_sha256", "publisher_build_manifest_sha256")))
        if not schema_gate:
            failures.append("accepted-sequence record %d has invalid typed values" % index)
            continue
        sample_envelope = (acceptance_samples[record["document_index"]]
                           if record["document_index"] < len(acceptance_samples)
                           else {})
        typed_all.append({
            "header_stamp_ns": record["header_stamp_ns"],
            "arrival_ros_ns": sample_envelope.get("arrival_ros_ns"),
            "arrival_steady_ns": sample_envelope.get("arrival_steady_ns"),
            "collector_sequence": sample_envelope.get("collector_sequence"),
            "session_id": values["session_id"], "reset_counter": reset_counter,
            "observation_sequence": observation_sequence,
            "accepted_sequence": accepted_sequence,
            "accepted": values["accepted"] == "true",
            "converged": values["converged"] == "true",
            "remote_timestamp_ns": remote_timestamp_ns,
            "round_trip_time_ms": rtt, "deviation_ns": deviation,
            "source_plugin_sha256": values["source_plugin_sha256"],
            "publisher_source_sha256": values["publisher_source_sha256"],
            "publisher_binary_sha256": values["publisher_binary_sha256"],
            "publisher_build_manifest_sha256": values[
                "publisher_build_manifest_sha256"],
            "values_raw": record["values_raw"],
        })

    sample_count_gate = (
        len(records) >= required and len(typed_all) == len(records))
    if not sample_count_gate:
        failures.append("typed accepted-sequence sample window is incomplete")
    typed = typed_all[-required:] if len(typed_all) >= required else typed_all
    stamps = [item["header_stamp_ns"] for item in typed_all]
    stamps_monotonic = bool(stamps) and all(
        right > left for left, right in zip(stamps, stamps[1:]))
    gap_gate = stamps_monotonic and all(
        right - left <= max_gap_ns for left, right in zip(stamps, stamps[1:]))
    fresh = bool(stamps) and 0 <= reference_ns - stamps[-1] <= max_age_ns
    if not stamps_monotonic or not gap_gate or not fresh:
        failures.append("typed accepted-sequence evidence is non-monotonic/stale")
    session_gate = (bool(typed_all) and
                    len({item["session_id"] for item in typed_all}) == 1)
    reset_gate = (bool(typed_all) and
                  len({item["reset_counter"] for item in typed_all}) == 1)
    observation_gate = bool(typed_all) and all(
        right > left for left, right in zip(
            [item["observation_sequence"] for item in typed_all],
            [item["observation_sequence"] for item in typed_all][1:]))
    accepted_sequences = [item["accepted_sequence"] for item in typed]
    all_accepted_sequences = [
        item["accepted_sequence"] for item in typed_all]
    accepted_no_rollback_gate = bool(typed_all) and all(
        right >= left for left, right in zip(
            all_accepted_sequences, all_accepted_sequences[1:]))
    accepted_transition_gate = bool(typed_all) and all(
        current["accepted_sequence"] == previous["accepted_sequence"] + (
            1 if current["accepted"] else 0)
        for previous, current in zip(typed_all, typed_all[1:]))
    accepted_sequence_gate = bool(typed) and all(
        right == left + 1 for left, right in zip(
            accepted_sequences, accepted_sequences[1:]))
    convergence_window = effective.get("time/convergence_window")
    convergence_gate = (
        _strict_int(convergence_window, 1) and bool(typed) and
        accepted_sequences[0] >= convergence_window and
        all(item["accepted"] and item["converged"] for item in typed))
    if not all((session_gate, reset_gate, observation_gate,
                accepted_no_rollback_gate,
                accepted_transition_gate, accepted_sequence_gate,
                convergence_gate)):
        failures.append("typed accepted-sequence continuity/convergence gate failed")

    source_gate = bool(typed_all) and all(
        item["source_plugin_sha256"] == source_hashes.get("mavros_timesync_plugin") and
        item["publisher_source_sha256"] == source_hashes.get(
            "timesync_acceptance_publisher_source") and
        item["publisher_binary_sha256"] == source_hashes.get(
            "timesync_acceptance_publisher_binary") and
        item["publisher_build_manifest_sha256"] == source_hashes.get(
            "timesync_acceptance_build_manifest") for item in typed_all)
    if not source_gate:
        failures.append("typed accepted-sequence source/build identity mismatch")

    max_rtt = effective.get("time/max_rtt_sample")
    max_deviation = effective.get("time/max_deviation_sample")
    residual_limit = int(config.get("max_abs_residual_ns", 2000000))
    status_by_remote = {}
    for index, envelope in enumerate(timesync_samples):
        sample = envelope.get("payload", {})
        remote = sample.get("remote_timestamp_ns")
        previous_estimate = (
            timesync_samples[index - 1].get("payload", {}).get(
                "estimated_offset_ns") if index > 0 else None)
        observed = sample.get("observed_offset_ns")
        residual = (abs(float(observed) - float(previous_estimate))
                    if (_strict_number(observed) and
                        _strict_number(previous_estimate)) else None)
        status_by_remote.setdefault(remote, []).append({
            "index": index, "envelope": envelope, "payload": sample,
            "residual_ns": residual})
    linked = []
    for item in typed:
        matches = status_by_remote.get(item["remote_timestamp_ns"], [])
        linked.append(matches[0] if len(matches) == 1 else None)
    expected_tail_indices = list(range(
        max(0, len(timesync_samples) - required), len(timesync_samples)))
    linked_indices = [item["index"] for item in linked if item is not None]
    tail_index_gate = (
        len(typed) == required and linked_indices == expected_tail_indices)
    max_link_delay_ns = int(float(config.get(
        "acceptance_link_max_delay_s", 0.5)) * 1.0e9)
    max_link_future_ns = int(float(config.get(
        "acceptance_link_max_future_s", 0.05)) * 1.0e9)
    linkage_gate = (
        tail_index_gate and all(
            link is not None and
            item["round_trip_time_ms"] == float(
                link["payload"].get("round_trip_time_ms")) and
            link["residual_ns"] is not None and
            abs(item["deviation_ns"]) == int(link["residual_ns"]) and
            _strict_int(item["arrival_steady_ns"], 1) and
            -max_link_future_ns <= (
                item["arrival_steady_ns"] -
                link["envelope"].get("arrival_steady_ns", 0)) <= max_link_delay_ns
            for item, link in zip(typed, linked)))
    threshold_gate = bool(typed) and all(
        _strict_number(max_rtt, 0) and item["round_trip_time_ms"] < float(max_rtt) and
        _strict_number(max_deviation, 0) and
        abs(item["deviation_ns"]) <= float(max_deviation) * 1000000.0 and
        abs(item["deviation_ns"]) <= residual_limit for item in typed)
    if not linkage_gate or not threshold_gate:
        failures.append("typed accepted-sequence sample linkage/threshold gate failed")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "matching_samples": len(records), "required_tail_samples": required,
        "levels": levels, "all_matching_levels_ok": all_levels_ok,
        "typed_records": typed, "typed_record_count": len(typed_all),
        "sample_count_gate": sample_count_gate,
        "header_stamps_monotonic": stamps_monotonic,
        "maximum_gap_gate": gap_gate, "fresh_at_finish_gate": fresh,
        "session_gate": session_gate, "reset_counter_gate": reset_gate,
        "observation_sequence_gate": observation_gate,
        "accepted_sequence_no_rollback_gate": accepted_no_rollback_gate,
        "accepted_sequence_transition_gate": accepted_transition_gate,
        "accepted_sequence_gate": accepted_sequence_gate,
        "explicit_converged_gate": convergence_gate,
        "source_build_identity_gate": source_gate,
        "timesync_sample_linkage_gate": linkage_gate,
        "timesync_tail_index_gate": tail_index_gate,
        "linked_timesync_indices": linked_indices,
        "maximum_arrival_link_delay_s": float(max_link_delay_ns) / 1.0e9,
        "strict_tail_threshold_gate": threshold_gate,
        "raw_statuses": parsed["raw_statuses"],
    }


def _timesync_report(documents, diagnostics, acceptance_documents, reference_ns,
                     diagnostic_reference_ns, acceptance_reference_ns,
                     config, effective, source_hashes, timesync_samples,
                     acceptance_samples):
    samples = int(config.get("samples", 40))
    tail_samples = int(config.get("tail_samples", 20))
    max_age_ns = int(float(config.get("max_age_s", 1.0)) * 1.0e9)
    max_gap_ns = int(float(config.get("max_gap_s", 0.5)) * 1.0e9)
    residual_limit = int(config.get("max_abs_residual_ns", 2000000))
    diagnostic_name = str(config.get("diagnostic_name", "mavros: Time Sync"))
    failures = []
    fields = ("remote_timestamp_ns", "observed_offset_ns",
              "estimated_offset_ns", "round_trip_time_ms")
    schema_ok = (
        len(documents) >= samples and tail_samples > 0 and len(documents) > tail_samples and
        all(_strict_int(item.get("remote_timestamp_ns"), 1) and
            _strict_int(item.get("observed_offset_ns")) and
            _strict_int(item.get("estimated_offset_ns")) and
            _strict_number(item.get("round_trip_time_ms"), 0)
            for item in documents))
    if not schema_ok:
        failures.append("timesync_status samples are incomplete or invalid")
    stamps = [_header_stamp_ns(item) for item in documents]
    stamp_valid = all(item is not None and item > 0 for item in stamps)
    monotonic = stamp_valid and all(
        right > left for left, right in zip(stamps, stamps[1:]))
    gap_gate = monotonic and all(
        right - left <= max_gap_ns for left, right in zip(stamps, stamps[1:]))
    fresh = bool(stamps) and stamp_valid and 0 <= reference_ns - stamps[-1] <= max_age_ns
    remotes = [item.get("remote_timestamp_ns") for item in documents]
    remote_monotonic = schema_ok and all(
        right > left for left, right in zip(remotes, remotes[1:]))
    if not monotonic or not gap_gate or not fresh or not remote_monotonic:
        failures.append("timesync clocks are not monotonic, continuous, and fresh")

    max_rtt_ms = effective.get("time/max_rtt_sample")
    max_deviation_ms = effective.get("time/max_deviation_sample")
    convergence_window = effective.get("time/convergence_window")
    tail = documents[-tail_samples:] if len(documents) >= tail_samples else []
    tail_start = len(documents) - len(tail)
    accepted = []
    residuals = []
    for index in range(tail_start, len(documents)):
        item = documents[index]
        previous_estimate = (documents[index - 1].get("estimated_offset_ns")
                             if index > 0 else None)
        observed = item.get("observed_offset_ns")
        residual = (abs(float(observed) - float(previous_estimate))
                    if (_strict_number(observed) and
                        _strict_number(previous_estimate)) else math.inf)
        residuals.append(residual if math.isfinite(residual) else None)
        accepted.append(
            _strict_number(max_rtt_ms, 0) and
            _strict_number(max_deviation_ms, 0) and
            _strict_number(item.get("round_trip_time_ms"), 0) and
            float(item["round_trip_time_ms"]) < float(max_rtt_ms) and
            residual <= float(max_deviation_ms) * 1000000.0 and
            residual <= float(residual_limit))
    tail_threshold_gate = len(accepted) == tail_samples and all(accepted)
    if not tail_threshold_gate:
        failures.append("timesync status tail violates strict source thresholds")

    diagnostic = _diagnostic_status(
        diagnostics, diagnostic_name, diagnostic_reference_ns,
        config.get("diagnostic_max_age_s", 2.0),
        config.get("diagnostic_max_gap_s", 2.0))
    diag_required = int(config.get("diagnostic_samples", 2))
    diagnostic_ok = diagnostic["ok"] and diagnostic["matching_samples"] >= diag_required
    if not diagnostic_ok:
        failures.append("timesync diagnostic window is not entirely OK/monotonic/fresh")
    acceptance = _timesync_acceptance_report(
        acceptance_documents, acceptance_reference_ns, config, effective,
        source_hashes, timesync_samples, acceptance_samples)
    if acceptance["status"] != "PASS":
        failures.append(
            "explicit source-bound accepted-sequence/convergence evidence failed")
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "sample_count": len(documents), "required_samples": samples,
        "tail_samples": tail_samples, "schema_valid": schema_ok,
        "header_stamps_monotonic": monotonic, "remote_stamps_monotonic": remote_monotonic,
        "maximum_gap_gate": gap_gate, "fresh_at_finish_gate": fresh,
        "tail_source_threshold_gate": tail_threshold_gate,
        "tail_strictly_accepted": acceptance["status"] == "PASS",
        "tail_previous_estimate_residual_ns": residuals,
        "strict_rtt_max_ms": max_rtt_ms,
        "maximum_acceptance_deviation_ms": max_deviation_ms,
        "maximum_contract_residual_ns": residual_limit,
        "diagnostic": diagnostic, "diagnostic_gate": diagnostic_ok,
        "accepted_sequence": acceptance,
        "explicit_accepted_sequence_gate": acceptance["status"] == "PASS",
        "diagnostic_observation_count_used_as_convergence": False,
        "summary": {field: _numeric_summary(documents, field) for field in fields},
    }


def _source_tree_value(document, key):
    value = document
    for part in key.split("/"):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def _rostopic_publisher_nodes(text):
    publishers = []
    in_publishers = False
    for line in str(text).splitlines():
        stripped = line.strip()
        if stripped == "Publishers:":
            in_publishers = True
            continue
        if stripped.endswith(":") and stripped != "Publishers:":
            in_publishers = False
        if not in_publishers or not stripped.startswith("*"):
            continue
        token = stripped[1:].strip().split()[0] if stripped[1:].strip() else ""
        if token.startswith("/"):
            publishers.append(token)
    return publishers


def _node_runtime_identity(text):
    pid_match = _PID_RE.search(str(text))
    uri_match = _NODE_URI_RE.search(str(text))
    uri = uri_match.group(1) if uri_match else None
    host = urlparse(uri).hostname if uri else None
    return {
        "pid": int(pid_match.group(1)) if pid_match else None,
        "uri": uri, "host": host, "local_host_gate": _local_host_gate(host),
    }


def _local_host_gate(host):
    if not isinstance(host, str) or not host:
        return False
    local_names = {"localhost", "127.0.0.1", "::1",
                   socket.gethostname(), socket.getfqdn()}
    if host in local_names:
        return True
    try:
        remote_addresses = {
            item[4][0] for item in socket.getaddrinfo(host, None)}
        local_addresses = {"127.0.0.1", "::1"}
        for name in (socket.gethostname(), socket.getfqdn()):
            local_addresses.update(
                item[4][0] for item in socket.getaddrinfo(name, None))
    except socket.gaierror:
        return False
    return bool(remote_addresses & local_addresses)


def _rosservice_provider_node(text):
    match = _SERVICE_NODE_RE.search(str(text))
    return match.group(1) if match else None


class FcuAuditCollector(object):
    """Collect one immutable audit run; incomplete evidence can never PASS."""

    def __init__(self, config, command_runner=None, pull_runner=None, clock_ns=None,
                 steady_clock_ns=None, process_maps_reader=None,
                 process_exe_reader=None):
        self.config = dict(config or {})
        _validate_audit_config(self.config)
        self.config_source_path = self.config["_config_source_path"]
        self.command_runner = command_runner or _default_runner
        self.pull_runner = pull_runner or self._default_pull_transaction
        self.clock_ns = clock_ns or time.time_ns
        self.steady_clock_ns = steady_clock_ns or time.monotonic_ns
        self.process_maps_reader = process_maps_reader or self._read_process_maps
        self.process_exe_reader = process_exe_reader or self._read_process_exe
        self.namespace = str(self.config.get("mavros_namespace", "/mavros")).rstrip("/")
        if not _NAMESPACE_RE.match(self.namespace):
            raise ValueError("invalid mavros_namespace")
        self.command_timeout_s = float(self.config.get("command_timeout_s", 15.0))
        self.param_timeout_s = float(self.config.get("param_timeout_s", 60.0))
        self.min_param_count = int(self.config.get("min_param_count", 100))
        self.trace_startup_s = float(self.config.get("trace_startup_s", 0.5))
        self.trace_tail_s = float(self.config.get("trace_tail_s", 0.5))
        self._source_hashes = {}
        self._trace_identity_authorized = False
        if any(not math.isfinite(item) or item <= 0 for item in (
                self.command_timeout_s, self.param_timeout_s)):
            raise ValueError("timeouts must be finite and positive")
        if (self.min_param_count < 1 or self.trace_startup_s < 0 or
                self.trace_tail_s < 0):
            raise ValueError("parameter/trace contract is invalid")
        for key in ("state_guard", "timesync", "estimator_status", "identity"):
            if not isinstance(self.config.get(key), dict):
                raise ValueError("%s config must be a mapping" % key)

    @staticmethod
    def _read_process_maps(pid):
        with open("/proc/%d/maps" % int(pid), "r") as stream:
            return stream.read()

    @staticmethod
    def _read_process_exe(pid):
        return os.path.realpath("/proc/%d/exe" % int(pid))

    def collect(self, output_root, run_id=None):
        run_id = run_id or default_run_id("fcu-audit")
        run = AppendOnlyRun(output_root, run_id)
        effective_config = dict(self.config)
        config_source_path = effective_config.pop("_config_source_path", None)
        effective_config.pop("_executed_collector_entrypoint", None)
        run.write_text(
            "evidence/effective_config.yaml",
            yaml.safe_dump(effective_config, default_flow_style=False, sort_keys=True),
            "application/x-yaml")
        started_at = utc_now()
        commands = []
        failures = []
        incomplete = []

        identity_report = self._collect_identity(run, commands, incomplete)
        self._trace_identity_authorized = identity_report["status"] == "PASS"

        sim_time_capture = self._capture(
            run, commands, "global_use_sim_time_precheck",
            ["rosparam", "get", "/use_sim_time"], self.command_timeout_s)
        try:
            use_sim_time = yaml.safe_load(sim_time_capture["stdout_text"])
        except yaml.YAMLError:
            use_sim_time = None
        global_clock_report = {
            "status": "PASS" if (
                sim_time_capture["command_ok"] and use_sim_time is False) else "FAIL",
            "expected_use_sim_time": False, "actual_use_sim_time": use_sim_time,
            "exact_false_gate": sim_time_capture["command_ok"] and
            use_sim_time is False,
        }
        self._global_clock_report = global_clock_report
        if not sim_time_capture["command_ok"]:
            incomplete.append("global /use_sim_time precheck is unavailable")
        elif global_clock_report["status"] != "PASS":
            failures.append(
                "global /use_sim_time must be explicitly false before FCU evidence")

        vehicle = self._capture(
            run, commands, "vehicle_info",
            ["rosservice", "call", self.namespace + "/vehicle_info_get",
             "{get_all: false, sysid: 0, compid: 0}"], self.command_timeout_s)
        vehicle_docs = _yaml_documents(vehicle["stdout_text"])
        vehicle_record = _first_vehicle(vehicle_docs)
        vehicle_report = self._vehicle_report(vehicle, vehicle_docs, vehicle_record)
        if vehicle_report["status"] != "PASS":
            if vehicle["command_ok"] or vehicle_record is not None:
                failures.extend(vehicle_report["failures"])
            else:
                incomplete.extend(vehicle_report["failures"])

        # These two one-shot samples deliberately occur after all metadata work
        # and immediately before the pull authorization decision.
        pre_state = self._capture_trace_topic(
            run, commands, "pre_state", "state", self.namespace + "/state", 2)
        pre_extended = self._capture_trace_topic(
            run, commands, "pre_extended_state", "extended_state",
            self.namespace + "/extended_state", 2)
        pre_state_trace = _trace_envelopes(pre_state["stdout_text"], "state")
        pre_extended_trace = _trace_envelopes(
            pre_extended["stdout_text"], "extended_state")
        pre_reference = max(pre_state["finished_wall_ns"], pre_extended["finished_wall_ns"])
        pre_guard = _phase_guard(
            pre_state_trace["samples"], pre_extended_trace["samples"],
            pre_reference, self.config["state_guard"], 2,
            pre_state_trace["ready_record"], pre_extended_trace["ready_record"],
            state_capture_start_steady_ns=pre_state["started_steady_ns"],
            state_capture_end_steady_ns=pre_state["finished_steady_ns"],
            extended_capture_start_steady_ns=pre_extended[
                "started_steady_ns"],
            extended_capture_end_steady_ns=pre_extended[
                "finished_steady_ns"])
        if (not pre_state["command_ok"] or not pre_extended["command_ok"] or
                not pre_state_trace["samples"] or
                not pre_extended_trace["samples"]):
            incomplete.append("fresh FCU precheck evidence is missing")
        if pre_state["stdout_text"].strip() and pre_state_trace["status"] != "PASS":
            failures.extend("precheck state trace: " + item
                            for item in pre_state_trace["failures"])
        if (pre_extended["stdout_text"].strip() and
                pre_extended_trace["status"] != "PASS"):
            failures.extend("precheck extended trace: " + item
                            for item in pre_extended_trace["failures"])
        failures.extend(
            "precheck: " + item for item in _measured_phase_failures(
                pre_guard, bool(pre_state_trace["samples"]),
                bool(pre_extended_trace["samples"])))

        param_relative = "evidence/px4_params_full.csv"
        param_path = run.path(param_relative)
        transaction = None
        transaction_captures = {}
        pre_pull_identity = self._source_identity_unchanged(identity_report)
        identity_report["pre_pull_source_recheck"] = pre_pull_identity
        if pre_pull_identity["status"] != "PASS":
            if identity_report["status"] == "PASS":
                failures.extend(pre_pull_identity["failures"])
            else:
                incomplete.extend(pre_pull_identity["failures"])
        if identity_report["status"] == "PASS":
            pre_pull_runtime = self._runtime_identity_unchanged(
                run, commands, identity_report, "pre_pull")
        else:
            pre_pull_runtime = {
                "status": "FAIL",
                "failures": [
                    "pre-pull runtime recheck skipped: initial identity failed"]}
        identity_report["pre_pull_runtime_recheck"] = pre_pull_runtime
        if pre_pull_runtime["status"] != "PASS":
            if identity_report["status"] == "PASS":
                failures.extend(pre_pull_runtime["failures"])
            else:
                incomplete.extend(pre_pull_runtime["failures"])
        pull_authorized = (
            pre_guard["status"] == "PASS" and
            pre_state_trace["status"] == "PASS" and
            pre_extended_trace["status"] == "PASS" and
            vehicle_report["status"] == "PASS" and
            global_clock_report["status"] == "PASS" and
            identity_report["status"] == "PASS" and
            pre_pull_identity["status"] == "PASS" and
            pre_pull_runtime["status"] == "PASS")
        if pull_authorized:
            source_paths = self.config["identity"].get("source_paths", {})
            trace_entrypoint = source_paths.get("trace_entrypoint", "")
            spec = {
                "dump_argv": [source_paths.get(
                                  "mavparam_python_interpreter", ""),
                              source_paths.get("mavparam_entrypoint", ""),
                              "-n", self.namespace,
                              "-v", "dump", param_path, "-f"],
                "param_trace_argv": [
                    trace_entrypoint, "--role", "param", "--topic",
                    self.namespace + "/param/param_value", "--count", "0",
                    "--connection-timeout", str(float(self.config.get(
                        "trace_connection_timeout_s", 10.0)))],
                "state_trace_argv": [
                    trace_entrypoint, "--role", "state", "--topic",
                    self.namespace + "/state", "--count", "0",
                    "--connection-timeout", str(float(self.config.get(
                        "trace_connection_timeout_s", 10.0)))],
                "extended_trace_argv": [
                    trace_entrypoint, "--role", "extended_state", "--topic",
                    self.namespace + "/extended_state", "--count", "0",
                    "--connection-timeout", str(float(self.config.get(
                        "trace_connection_timeout_s", 10.0)))],
                "trace_startup_s": self.trace_startup_s,
                "trace_tail_s": self.trace_tail_s,
            }
            transaction = self.pull_runner(spec, self.param_timeout_s)
            for label, key, argv in (
                    ("full_param_dump", "dump", spec["dump_argv"]),
                    ("raw_param_trace", "param_trace", spec["param_trace_argv"]),
                    ("during_state", "state_trace", spec["state_trace_argv"]),
                    ("during_extended_state", "extended_trace", spec["extended_trace_argv"])):
                capture = self._record_capture(
                    run, label, argv, transaction.get(key, _result(error="missing result")),
                    self.param_timeout_s)
                commands.append(capture)
                transaction_captures[key] = capture
            if not transaction.get("ready_handshake", False):
                failures.append(
                    "pull transaction: all trace subscribers were not ready "
                    "before force pull")
            if not transaction.get("safety_sample_handshake", False):
                failures.append(
                    "pull transaction: state/extended trace lacked a safe "
                    "actual-arrival sample before pull")
            if not transaction.get("concurrent", False):
                failures.append(
                    "pull transaction: parameter/state raw traces did not remain "
                    "concurrent with pull")
        else:
            incomplete.append(
                "forced parameter pull skipped because fresh safe precheck, vehicle, "
                "package/message/source, or bundle identity failed")

        post_state = self._capture_trace_topic(
            run, commands, "post_state", "state", self.namespace + "/state", 2)
        post_extended = self._capture_trace_topic(
            run, commands, "post_extended_state", "extended_state",
            self.namespace + "/extended_state", 2)
        post_state_trace = _trace_envelopes(post_state["stdout_text"], "state")
        post_extended_trace = _trace_envelopes(
            post_extended["stdout_text"], "extended_state")
        post_reference = max(post_state["finished_wall_ns"], post_extended["finished_wall_ns"])
        post_guard = _phase_guard(
            post_state_trace["samples"], post_extended_trace["samples"],
            post_reference, self.config["state_guard"], 2,
            post_state_trace["ready_record"], post_extended_trace["ready_record"],
            state_capture_start_steady_ns=post_state["started_steady_ns"],
            state_capture_end_steady_ns=post_state["finished_steady_ns"],
            extended_capture_start_steady_ns=post_extended[
                "started_steady_ns"],
            extended_capture_end_steady_ns=post_extended[
                "finished_steady_ns"])
        if (not post_state["command_ok"] or not post_extended["command_ok"] or
                not post_state_trace["samples"] or
                not post_extended_trace["samples"]):
            incomplete.append("fresh FCU postcheck evidence is missing")
        if post_state["stdout_text"].strip() and post_state_trace["status"] != "PASS":
            failures.extend("postcheck state trace: " + item
                            for item in post_state_trace["failures"])
        if (post_extended["stdout_text"].strip() and
                post_extended_trace["status"] != "PASS"):
            failures.extend("postcheck extended trace: " + item
                            for item in post_extended_trace["failures"])
        failures.extend(
            "postcheck: " + item for item in _measured_phase_failures(
                post_guard, bool(post_state_trace["samples"]),
                bool(post_extended_trace["samples"])))

        during_guard = {
            "status": "FAIL", "failures": ["parameter pull did not run"],
            "state_samples": 0, "extended_state_samples": 0,
        }
        param_trace_report = {
            "status": "FAIL", "failures": ["parameter pull did not run"],
            "raw_message_count": 0,
        }
        parameters = {}
        dump_errors = []
        param_artifact = None
        reported_param_count = None
        if transaction is not None:
            during_state = transaction_captures["state_trace"]
            during_extended = transaction_captures["extended_trace"]
            dump_capture = transaction_captures["dump"]
            transaction_reference = int(transaction.get("finished_wall_ns", post_reference))
            during_state_trace = _trace_envelopes(
                during_state["stdout_text"], "state")
            during_extended_trace = _trace_envelopes(
                during_extended["stdout_text"], "extended_state")
            during_guard = _phase_guard(
                during_state_trace["samples"],
                during_extended_trace["samples"], transaction_reference,
                self.config["state_guard"],
                int(self.config["state_guard"].get("during_min_samples", 3)),
                during_state_trace["ready_record"],
                during_extended_trace["ready_record"],
                dump_capture.get("started_steady_ns"),
                dump_capture.get("finished_steady_ns"))
            if (not during_state["command_ok"] or not during_extended["command_ok"] or
                    during_state_trace["status"] != "PASS" or
                    during_extended_trace["status"] != "PASS" or
                    during_guard["status"] != "PASS"):
                failures.extend("during pull trace: " + item for item in
                                during_state_trace["failures"] +
                                during_extended_trace["failures"])
                failures.extend("during pull: " + item for item in during_guard["failures"])
            param_text = ""
            if os.path.lexists(param_path):
                try:
                    param_artifact = run.adopt_existing(param_relative, "text/csv")
                    with open(param_path, "r") as stream:
                        param_text = stream.read()
                except (OSError, ValueError) as exc:
                    incomplete.append("parameter dump artifact invalid: %s" % exc)
            parameters, dump_errors = _parse_param_dump(param_text)
            match = _PARAM_COUNT_RE.search(dump_capture["stdout_text"])
            reported_param_count = int(match.group(1)) if match else None
            param_trace = _trace_envelopes(
                transaction_captures["param_trace"]["stdout_text"], "param")
            param_trace_report = _analyze_param_trace(
                param_trace["samples"],
                parameters, reported_param_count,
                vehicle_report.get("autopilot"), self.min_param_count,
                self.config["state_guard"], param_trace["ready_record"],
                dump_capture.get("started_steady_ns"),
                dump_capture.get("finished_steady_ns"))
            if not dump_capture["command_ok"] or param_artifact is None:
                incomplete.append("forced full PX4 parameter dump/raw trace is incomplete")
            if dump_errors or param_trace["status"] != "PASS" or \
                    param_trace_report["status"] != "PASS":
                failures.extend("parameter dump: " + item for item in dump_errors)
                failures.extend("parameter trace envelope: " + item for item in
                                param_trace["failures"])
                failures.extend("parameter trace: " + item for item in
                                param_trace_report["failures"])

        (timesync_documents, diagnostic_documents, acceptance_documents,
         timesync_report, effective_timesync) = self._collect_timesync(
             run, commands, incomplete, failures)

        estimator_count = int(self.config["estimator_status"].get("samples", 10))
        estimator_capture = self._capture_topic(
            run, commands, "estimator_status", self.namespace + "/estimator_status",
            estimator_count)
        estimator_documents = _yaml_documents(estimator_capture["stdout_text"])
        estimator_report = _estimator_report(
            estimator_documents, estimator_capture["finished_wall_ns"],
            self.config["estimator_status"])
        if not estimator_capture["command_ok"] or not estimator_documents:
            incomplete.append("estimator_status continuous contract is incomplete")
        if (estimator_capture["stdout_text"].strip() and
                not estimator_documents):
            failures.append(
                "estimator_status: successful capture returned malformed evidence")
        elif estimator_documents and estimator_report["status"] != "PASS":
            failures.extend(
                "estimator_status: " + item for item in estimator_report["failures"])

        if identity_report["status"] == "PASS":
            post_runtime = self._runtime_identity_unchanged(
                run, commands, identity_report, "post_evidence")
            post_sources = self._source_identity_unchanged(identity_report)
        else:
            post_runtime = {
                "status": "FAIL", "failures": [
                    "post-evidence runtime recheck skipped: initial identity failed"]}
            post_sources = {
                "status": "FAIL", "failures": [
                    "post-evidence source recheck skipped: initial identity failed"]}
        identity_report["post_evidence_runtime_recheck"] = post_runtime
        identity_report["post_evidence_source_recheck"] = post_sources
        if identity_report["status"] == "PASS":
            failures.extend(post_runtime["failures"])
            failures.extend(post_sources["failures"])

        inventory = {
            name: {"available": name in parameters, "captured_value": parameters.get(name)}
            for name in PARAMETER_INVENTORY
        }
        if failures:
            status = "FAIL"
        elif incomplete:
            status = "INCOMPLETE"
        else:
            status = "PASS"
        document = {
            "schema": SCHEMA, "run_id": run_id,
            "started_at_utc": started_at, "finished_at_utc": utc_now(),
            "collection_mode": "read_only_fcu_audit",
            "flight_authority": "NONE", "status": status, "fail_closed": True,
            "status_interpretation": (
                "PASS proves only this read-only evidence contract. It never authorizes "
                "parameter changes, estimator fusion, arming, offboard, or flight."),
            "failures": failures, "incomplete_evidence": incomplete,
            "safety_invariants": {
                "application_ros_topic_publishers": False,
                "sets_persistent_fcu_parameters": False,
                "arms_or_changes_mode": False,
                "sends_mavlink_commands_or_setpoints": False,
                "forced_pull_reads_persistent_fcu_parameters": True,
                "forced_pull_refreshes_mavros_ros_parameter_cache": True,
                "forced_pull_cache_side_effect": (
                    "mavparam dump -f calls MAVROS param/pull(force_pull=true); MAVROS "
                    "updates its ROS parameter cache from FCU values but does not persistently "
                    "write FCU parameters."),
            },
            "mavros_namespace": self.namespace,
            "collection_contract": effective_config,
            "implementation_identity": identity_report,
            "guards": {"pre": pre_guard, "during_pull": during_guard, "post": post_guard},
            "observations": {
                "vehicle": vehicle_report,
                "global_clock_domain": global_clock_report,
                "forced_pull_transaction": ({
                    "ready_handshake": bool(transaction.get("ready_handshake")),
                    "safety_sample_handshake": bool(transaction.get(
                        "safety_sample_handshake")),
                    "concurrent_through_tail": bool(transaction.get("concurrent")),
                    "started_wall_ns": transaction.get("started_wall_ns"),
                    "finished_wall_ns": transaction.get("finished_wall_ns"),
                    "started_steady_ns": transaction.get("started_steady_ns"),
                    "finished_steady_ns": transaction.get("finished_steady_ns"),
                } if isinstance(transaction, dict) else {
                    "ready_handshake": False,
                    "safety_sample_handshake": False,
                    "concurrent_through_tail": False,
                    "started_wall_ns": None, "finished_wall_ns": None,
                    "started_steady_ns": None, "finished_steady_ns": None}),
                "parameter_trace": param_trace_report,
                "parameter_dump_artifact": param_artifact,
                "parameter_dump_parse_errors": dump_errors,
                "parameter_inventory": inventory,
                "parameter_interface": (
                    "EKF2_EV_CTRL" if "EKF2_EV_CTRL" in parameters else
                    "EKF2_AID_MASK" if "EKF2_AID_MASK" in parameters else "unavailable"),
                "timesync_effective_config": effective_timesync,
                "timesync": timesync_report,
                "timesync_raw_samples": len(timesync_documents),
                "timesync_diagnostic_raw_samples": len(diagnostic_documents),
                "timesync_accepted_sequence_raw_samples": len(
                    acceptance_documents),
                "estimator_status": estimator_report,
            },
            "limitations": {
                "battery_evidence": (
                    "not captured or gated: vehicle-specific battery topic, cell count, "
                    "chemistry, load-sag model, and thresholds are not frozen"),
                "runtime_identity_scope": (
                    "PASS binds the observed MAVROS process' mapped param-plugin library to "
                    "the supplied full source-tree/build manifests and command entrypoints; "
                    "it does not qualify any different deployment or future rebuild"),
            },
            "commands": [{key: value for key, value in item.items()
                          if key != "stdout_text"} for item in commands],
        }
        receipt_path, receipt = run.finalize(document)
        return receipt_path, receipt

    def _vehicle_report(self, capture, documents, vehicle):
        config = self.config["identity"]
        failures = []
        success = next((item.get("success") for item in documents
                        if isinstance(item.get("success"), bool)), None)
        if not capture["command_ok"] or success is not True or vehicle is None:
            failures.append("vehicle_info is unavailable or unsuccessful")
            return {"status": "FAIL", "failures": failures, "record": vehicle,
                    "autopilot": None}
        available = vehicle.get("available_info")
        numeric_fields = {
            "available_info": (0, 255), "sysid": (1, 255), "compid": (1, 255),
            "autopilot": (0, 255), "type": (0, 255),
            "flight_sw_version": (1, 0xffffffff), "uid": (0, None),
        }
        for name, bounds in numeric_fields.items():
            if not _strict_int(vehicle.get(name), bounds[0], bounds[1]):
                failures.append("vehicle_info %s is missing/non-numeric (bool rejected)" % name)
        optional_numeric_fields = {
            "system_status": (0, 255), "base_mode": (0, 255),
            "custom_mode": (0, 0xffffffff), "mode_id": (0, 0xffffffff),
            "capabilities": (0, None), "middleware_sw_version": (0, 0xffffffff),
            "os_sw_version": (0, 0xffffffff), "board_version": (0, 0xffffffff),
            "vendor_id": (0, 65535), "product_id": (0, 65535)}
        for name, bounds in optional_numeric_fields.items():
            value = vehicle.get(name)
            if value is not None and not _strict_int(value, bounds[0], bounds[1]):
                failures.append("vehicle_info %s has invalid numeric type (bool rejected)" % name)
        if _strict_int(available) and (available & 2) != 2:
            failures.append("vehicle_info lacks AUTOPILOT_VERSION evidence")
        required_autopilot = int(config.get("required_autopilot", MAV_AUTOPILOT_PX4))
        if vehicle.get("autopilot") != required_autopilot:
            failures.append("vehicle autopilot does not match required MAV_AUTOPILOT")
        custom = vehicle.get("flight_custom_version")
        if not isinstance(custom, str) or not _HEX_VERSION_RE.match(custom):
            failures.append("vehicle flight_custom_version is absent/non-hex")
        if bool(config.get("require_vehicle_uid", True)) and not _strict_int(vehicle.get("uid"), 1):
            failures.append("vehicle UID is required and unavailable")
        record_fields = (
            "available_info", "sysid", "compid", "autopilot", "type",
            "system_status", "base_mode", "custom_mode", "mode", "mode_id",
            "capabilities", "flight_sw_version", "middleware_sw_version",
            "os_sw_version", "board_version", "flight_custom_version",
            "vendor_id", "product_id", "uid")
        return {
            "status": "PASS" if not failures else "FAIL", "failures": failures,
            "autopilot": vehicle.get("autopilot"),
            "required_autopilot": required_autopilot,
            "record": {key: vehicle.get(key) for key in record_fields},
        }

    def _collect_timesync(self, run, commands, incomplete, failures):
        config = self.config["timesync"]
        effective_expected = dict(config.get("effective_config", {}))
        effective = {}
        effective_evidence = {}
        global_clock = getattr(self, "_global_clock_report", None)
        use_sim_time = (global_clock.get("actual_use_sim_time")
                        if isinstance(global_clock, dict) else None)
        use_sim_time_gate = (
            isinstance(global_clock, dict) and
            global_clock.get("status") == "PASS")
        effective_evidence["/use_sim_time"] = {
            "expected": False, "actual": use_sim_time,
            "exact": use_sim_time_gate}
        for key, expected in sorted(effective_expected.items()):
            capture = self._capture(
                run, commands, "timesync_config_" + key.replace("/", "_"),
                ["rosparam", "get", self.namespace + "/" + key],
                self.command_timeout_s)
            try:
                value = yaml.safe_load(capture["stdout_text"])
            except yaml.YAMLError:
                value = None
            exact = capture["command_ok"] and type(value) is type(expected) and value == expected
            if isinstance(expected, float) and _strict_number(value):
                exact = float(value) == expected
            effective[key] = value
            effective_evidence[key] = {"expected": expected, "actual": value, "exact": exact}
            if not exact:
                if capture["command_ok"]:
                    failures.append(
                        "effective MAVROS timesync config mismatch: %s" % key)
                else:
                    incomplete.append(
                        "effective MAVROS timesync config unavailable: %s" % key)
        samples = int(config.get("samples", 40))
        diagnostic_count = int(config.get("diagnostic_samples", 2))
        acceptance_count = int(config.get(
            "acceptance_samples", config.get("tail_samples", 20)))
        acceptance_topic = str(config.get(
            "acceptance_topic", "/flight_safety/mavros_timesync_accepted_sequence"))
        trace_path = self.config["identity"].get("source_paths", {}).get(
            "timesync_trace_entrypoint", "")
        capture_timeout = float(config.get("capture_timeout_s", 15.0))
        connection_timeout = float(config.get("connection_timeout_s", 10.0))
        tail_grace = float(config.get("tail_grace_s", 0.5))
        trace_argv = [trace_path,
             "--timesync-topic", self.namespace + "/timesync_status",
             "--diagnostics-topic", "/diagnostics",
             "--acceptance-topic", acceptance_topic,
             "--timesync-min-samples", str(samples),
             "--diagnostic-min-samples", str(diagnostic_count),
             "--acceptance-min-samples", str(acceptance_count),
             "--diagnostic-status-name", str(config["diagnostic_name"]),
             "--acceptance-status-name", str(config["acceptance_status_name"]),
             "--connection-timeout", str(connection_timeout),
             "--capture-timeout", str(capture_timeout),
             "--tail-grace", str(tail_grace)]
        trace_capture = self._capture_identity_bound_helper(
            run, commands, "timesync_concurrent_trace",
            "timesync_trace_entrypoint", trace_argv,
            connection_timeout + capture_timeout + tail_grace + 2.0)
        expected_topics = {
            "timesync_status": self.namespace + "/timesync_status",
            "diagnostics": "/diagnostics",
            "accepted_sequence": acceptance_topic,
        }
        minimums = {
            "timesync_status": samples, "diagnostics": diagnostic_count,
            "accepted_sequence": acceptance_count,
        }
        freshness = {
            "timesync_status": {
                "max_age_s": config.get("max_age_s", 1.0),
                "max_future_s": config.get("max_future_s", 0.05)},
            "diagnostics": {
                "max_age_s": config.get("diagnostic_max_age_s", 2.0),
                "max_future_s": config.get("max_future_s", 0.05)},
            "accepted_sequence": {
                "max_age_s": config.get("acceptance_max_age_s", 1.0),
                "max_future_s": config.get("max_future_s", 0.05)},
        }
        trace_report = _timesync_trace_envelopes(
            trace_capture["stdout_text"], expected_topics, minimums, freshness)
        trace_samples = trace_report["samples"]
        documents = _trace_payloads({"samples": trace_samples["timesync_status"]})
        diagnostic_documents = _trace_payloads(
            {"samples": trace_samples["diagnostics"]})
        acceptance_documents = _trace_payloads(
            {"samples": trace_samples["accepted_sequence"]})
        report = _timesync_report(
            documents, diagnostic_documents, acceptance_documents,
            trace_capture["finished_wall_ns"], trace_capture["finished_wall_ns"],
            trace_capture["finished_wall_ns"], config, effective,
            self._source_hashes, trace_samples["timesync_status"],
            trace_samples["accepted_sequence"])
        report["concurrent_trace"] = {
            key: value for key, value in trace_report.items()
            if key not in ("samples", "ready_record")}
        if not trace_capture["command_ok"]:
            incomplete.append(
                "concurrent timesync/diagnostic/accepted-sequence evidence is incomplete")
            ignored_absence_failures = {
                "timesync trace does not meet every stream sample minimum",
                "timesync trace stream windows do not overlap"}
            failures.extend(
                "timesync measured concurrent trace: " + item
                for item in trace_report["failures"]
                if item not in ignored_absence_failures)
            diagnostic_report = report["diagnostic"]
            if trace_samples["diagnostics"]:
                failures.extend(
                    "timesync measured diagnostic: " + item
                    for item in diagnostic_report["diagnostic_parse_failures"])
                if diagnostic_report["matching_samples"]:
                    for gate, description in (
                            ("all_diagnostic_status_schema_gate", "schema"),
                            ("all_matching_levels_ok", "level"),
                            ("counter_schema_valid", "counter schema"),
                            ("counter_no_rollback_gate", "counter rollback"),
                            ("header_stamps_monotonic", "header monotonicity"),
                            ("maximum_gap_gate", "maximum gap"),
                            ("fresh_at_finish_gate", "finish freshness")):
                        if not diagnostic_report[gate]:
                            failures.append(
                                "timesync measured diagnostic %s gate failed" %
                                description)
            if trace_samples["accepted_sequence"]:
                failures.extend(
                    "timesync measured acceptance: " + item
                    for item in report["accepted_sequence"]["failures"])
            if len(trace_samples["timesync_status"]) >= int(config["tail_samples"]):
                if not report["header_stamps_monotonic"]:
                    failures.append(
                        "timesync measured status headers are non-monotonic")
                if not report["remote_stamps_monotonic"]:
                    failures.append(
                        "timesync measured remote stamps are non-monotonic")
                if not report["tail_source_threshold_gate"]:
                    failures.append(
                        "timesync measured status tail violates thresholds")
        elif trace_report["status"] != "PASS" or report["status"] != "PASS":
            failures.extend(
                "timesync concurrent trace: " + item
                for item in trace_report["failures"])
            failures.extend("timesync: " + item for item in report["failures"])
        source_identity = self._timesync_source_config_gate(effective)
        effective_evidence["source_config_gate"] = source_identity
        if source_identity["status"] != "PASS":
            if source_identity.get("source_available", False):
                failures.extend(source_identity["failures"])
            else:
                incomplete.extend(source_identity["failures"])
        return (documents, diagnostic_documents, acceptance_documents,
                report, effective_evidence)

    def _timesync_source_config_gate(self, effective):
        path = self.config["identity"].get("source_paths", {}).get(
            "mavros_timesync_config")
        failures = []
        document = None
        source_sha256 = None
        identity_sha256 = self._source_hashes.get("mavros_timesync_config")
        if isinstance(path, str) and os.path.isabs(path) and os.path.isfile(path):
            try:
                source_sha256 = sha256_file(path)
                with open(path, "r") as stream:
                    document = yaml.safe_load(stream)
            except (OSError, yaml.YAMLError):
                document = None
        if not isinstance(document, dict):
            failures.append("MAVROS timesync config source is unavailable")
        if source_sha256 is None or source_sha256 != identity_sha256:
            failures.append("MAVROS timesync config changed after identity capture")
        matches = {}
        for key, actual in sorted(effective.items()):
            source_value = _source_tree_value(document, key) if document is not None else None
            exact = type(source_value) is type(actual) and source_value == actual
            if isinstance(actual, float) and _strict_number(source_value):
                exact = float(source_value) == actual
            matches[key] = {"source": source_value, "effective": actual, "exact": exact}
            if not exact:
                failures.append("timesync source/effective mismatch: %s" % key)
        return {"status": "PASS" if not failures else "FAIL",
                "failures": failures, "matches": matches,
                "source_sha256": source_sha256,
                "identity_capture_sha256": identity_sha256,
                "source_available": isinstance(document, dict) and
                source_sha256 is not None and identity_sha256 is not None}

    def _collect_identity(self, run, commands, incomplete):
        config = self.config["identity"]
        package_reports = {}
        expected_version = str(config.get("required_mavros_version", ""))
        for package in ("mavros", "mavros_msgs"):
            version_capture = self._capture(
                run, commands, package + "_version", ["rosversion", package],
                self.command_timeout_s)
            path_capture = self._capture(
                run, commands, package + "_package_path", ["rospack", "find", package],
                self.command_timeout_s)
            version = version_capture["stdout_text"].strip()
            package_path = path_capture["stdout_text"].strip()
            ok = (version_capture["command_ok"] and path_capture["command_ok"] and
                  bool(version) and os.path.isabs(package_path) and
                  os.path.isdir(package_path))
            if expected_version:
                ok = ok and version == expected_version
            package_reports[package] = {
                "version": version or None, "package_path": package_path or None,
                "required_version": expected_version or None, "gate": ok,
            }
            if not ok:
                incomplete.append("%s package/version identity is incomplete" % package)

        message_reports = {}
        for message_type, expected_md5 in sorted(EXPECTED_MESSAGE_IDENTITIES.items()):
            capture = self._capture(
                run, commands, "message_md5_" + message_type.replace("/", "_"),
                ["rosmsg", "md5", message_type], self.command_timeout_s)
            actual = capture["stdout_text"].strip()
            gate = capture["command_ok"] and actual == expected_md5
            message_reports[message_type] = {
                "expected_md5": expected_md5, "actual_md5": actual or None, "gate": gate}
            if not gate:
                incomplete.append("message MD5 mismatch: %s" % message_type)

        acceptance_topic = str(self.config["timesync"].get(
            "acceptance_topic", "/flight_safety/mavros_timesync_accepted_sequence"))
        topic_types = {
            self.namespace + "/param/param_value": "mavros_msgs/Param",
            self.namespace + "/timesync_status": "mavros_msgs/TimesyncStatus",
            self.namespace + "/estimator_status": "mavros_msgs/EstimatorStatus",
            self.namespace + "/state": "mavros_msgs/State",
            self.namespace + "/extended_state": "mavros_msgs/ExtendedState",
            "/diagnostics": "diagnostic_msgs/DiagnosticArray",
            acceptance_topic: "diagnostic_msgs/DiagnosticArray",
        }
        topic_reports = {}
        for topic, expected_type in sorted(topic_types.items()):
            capture = self._capture(
                run, commands, "topic_type_" + topic.strip("/").replace("/", "_"),
                ["rostopic", "type", topic], self.command_timeout_s)
            actual = capture["stdout_text"].strip()
            gate = capture["command_ok"] and actual == expected_type
            topic_reports[topic] = {
                "expected_type": expected_type, "actual_type": actual or None, "gate": gate}
            if not gate:
                incomplete.append("runtime topic type mismatch: %s" % topic)

        source_paths = dict(config.get("source_paths", {}))
        source_paths["fcu_audit_config"] = self.config_source_path
        local_labels = (
            "fcu_audit_config", "collector_entrypoint", "trace_entrypoint",
            "timesync_trace_entrypoint", "bundle_manifest")
        external_labels = (
            "mavparam_entrypoint", "mavparam_python_interpreter",
            "mavros_param_python", "mavros_param_runtime_module",
            "mavros_param_plugin",
            "mavros_param_plugin_binary", "mavros_param_source_tree_manifest",
            "mavros_param_build_manifest", "mavros_build_cmake_cache",
            "mavros_build_marker", "mavros_timesync_plugin",
            "mavros_timesync_config", "timesync_acceptance_publisher_source",
            "timesync_acceptance_publisher_binary",
            "timesync_acceptance_build_manifest")
        source_reports = {}
        capture_limit = int(config.get("source_artifact_max_bytes", 2 * 1024 * 1024))
        for label in local_labels + external_labels:
            path = source_paths.get(label)
            gate = (isinstance(path, str) and os.path.isabs(path) and
                    os.path.isfile(path) and not os.path.islink(path))
            artifact = None
            digest = None
            size = None
            if gate:
                size = os.path.getsize(path)
                digest = sha256_file(path)
                if size <= capture_limit:
                    with open(path, "rb") as stream:
                        artifact = run.write_bytes(
                            "evidence/sources/%s%s" %
                            (label, os.path.splitext(path)[1]),
                            stream.read(), "application/octet-stream")
            else:
                incomplete.append("required source identity is unavailable: %s" % label)
            source_reports[label] = {
                "path": path, "realpath": os.path.realpath(path)
                if isinstance(path, str) and path else None,
                "size_bytes": size, "sha256": digest, "artifact": artifact,
                "artifact_omitted_due_to_size": bool(
                    gate and artifact is None and size > capture_limit),
                "gate": gate}
            self._source_hashes[label] = digest

        policy_document = None
        try:
            with open(self.config_source_path, "r") as stream:
                policy_document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError):
            policy_document = None
        configured_policy = {
            key: value for key, value in self.config.items()
            if key not in ("_config_source_path", "_executed_collector_entrypoint")}
        policy_content_gate = (
            isinstance(policy_document, dict) and
            policy_document == configured_policy)
        source_reports["fcu_audit_config"]["effective_content_gate"] = (
            policy_content_gate)
        source_reports["fcu_audit_config"]["gate"] = (
            source_reports["fcu_audit_config"]["gate"] and
            policy_content_gate)
        if not policy_content_gate:
            incomplete.append(
                "actual FCU audit policy file does not exactly equal effective config")

        core_sources = {}
        for label, path in (
                ("collector_core", os.path.realpath(__file__)),
                ("append_only_evidence", os.path.join(
                    os.path.dirname(os.path.realpath(__file__)),
                    "append_only_evidence.py"))):
            gate = os.path.isfile(path) and not os.path.islink(path)
            core_sources[label] = {
                "path": path, "sha256": sha256_file(path) if gate else None,
                "gate": gate}
            if not gate:
                incomplete.append("collector core source identity unavailable: %s" % label)

        executed_entrypoint = self.config.get("_executed_collector_entrypoint")
        collector_entrypoint_gate = (
            isinstance(executed_entrypoint, str) and
            os.path.isabs(executed_entrypoint) and
            os.path.realpath(executed_entrypoint) ==
            source_reports["collector_entrypoint"]["realpath"])
        if not collector_entrypoint_gate:
            incomplete.append(
                "configured collector entrypoint is not the actual executed entrypoint")

        mavros_root = package_reports.get("mavros", {}).get("package_path")
        expected_runtime_paths = {}
        if isinstance(mavros_root, str) and os.path.isabs(mavros_root):
            expected_runtime_paths = {
                "mavparam_entrypoint": os.path.join(mavros_root, "scripts", "mavparam"),
                "mavros_param_python": os.path.join(
                    mavros_root, "src", "mavros", "param.py"),
                "mavros_param_plugin": os.path.join(
                    mavros_root, "src", "plugins", "param.cpp"),
                "mavros_timesync_plugin": os.path.join(
                    mavros_root, "src", "plugins", "sys_time.cpp"),
                "mavros_timesync_config": os.path.join(
                    mavros_root, "launch", "px4_config.yaml"),
            }
        runtime_path_gates = {
            label: (bool(expected_runtime_paths) and
                    os.path.realpath(source_reports[label]["path"] or "") ==
                    os.path.realpath(expected_path))
            for label, expected_path in expected_runtime_paths.items()}
        if set(runtime_path_gates) != {
                "mavparam_entrypoint", "mavros_param_python",
                "mavros_param_plugin", "mavros_timesync_plugin",
                "mavros_timesync_config"}:
            runtime_path_gates = {
                label: False for label in (
                    "mavparam_entrypoint", "mavros_param_python",
                    "mavros_param_plugin", "mavros_timesync_plugin",
                    "mavros_timesync_config")}
        if not all(runtime_path_gates.values()):
            incomplete.append(
                "mavparam/param.py/param-plugin source do not belong to runtime mavros tree")

        module_probe_code = (
            "import hashlib,json,mavros.param,os,sys;"
            "p=os.path.realpath(mavros.param.__file__);"
            "h=hashlib.sha256(open(p,'rb').read()).hexdigest();"
            "print(json.dumps({'interpreter':os.path.realpath(sys.executable),"
            "'module_path':p,'module_sha256':h},sort_keys=True))")
        interpreter_path = source_reports[
            "mavparam_python_interpreter"]["realpath"]
        module_probe = self._capture(
            run, commands, "mavparam_python_runtime_module",
            [interpreter_path or "", "-c", module_probe_code],
            self.command_timeout_s)
        try:
            module_probe_document = json.loads(module_probe["stdout_text"])
        except (TypeError, ValueError):
            module_probe_document = None
        configured_runtime_module = source_reports[
            "mavros_param_runtime_module"]
        module_probe_gate = (
            module_probe["command_ok"] and
            isinstance(module_probe_document, dict) and
            set(module_probe_document) == {
                "interpreter", "module_path", "module_sha256"} and
            os.path.realpath(module_probe_document.get("interpreter", "")) ==
            interpreter_path and
            os.path.realpath(module_probe_document.get("module_path", "")) ==
            configured_runtime_module["realpath"] and
            module_probe_document.get("module_sha256") ==
            configured_runtime_module["sha256"] ==
            source_reports["mavros_param_python"]["sha256"])
        if not module_probe_gate:
            incomplete.append(
                "mavparam interpreter/imported mavros.param module identity failed")

        def load_json_source(label):
            path = source_reports[label]["path"]
            if not source_reports[label]["gate"]:
                return None
            try:
                with open(path, "r") as stream:
                    value = json.load(stream)
            except (OSError, ValueError):
                return None
            return value if isinstance(value, dict) else None

        tree_document = load_json_source("mavros_param_source_tree_manifest")
        tree_failures = []
        tree_files = tree_document.get("files") if isinstance(tree_document, dict) else None
        tree_top_gate = (
            isinstance(tree_document, dict) and
            set(tree_document) == {"schema", "package_root", "files"} and
            tree_document.get("schema") ==
            "flight_safety/mavros_param_source_tree/v1" and
            isinstance(tree_document.get("package_root"), str) and
            os.path.realpath(tree_document.get("package_root")) ==
            os.path.realpath(mavros_root or "") and isinstance(tree_files, dict) and
            bool(tree_files))
        actual_tree_files = {}
        tree_special_entries = []
        if isinstance(mavros_root, str) and os.path.isdir(mavros_root):
            for root, dirs, files in os.walk(mavros_root):
                tree_special_entries.extend(
                    os.path.relpath(os.path.join(root, item), mavros_root)
                    for item in dirs if os.path.islink(os.path.join(root, item)))
                dirs[:] = sorted(
                    item for item in dirs
                    if item not in (".git", "__pycache__") and
                    not os.path.islink(os.path.join(root, item)))
                for filename in sorted(files):
                    path = os.path.join(root, filename)
                    if os.path.islink(path):
                        tree_special_entries.append(os.path.relpath(path, mavros_root))
                        continue
                    if filename.endswith((".pyc", ".pyo")):
                        continue
                    relative = os.path.relpath(path, mavros_root).replace(os.sep, "/")
                    actual_tree_files[relative] = sha256_file(path)
        tree_files_gate = (
            tree_top_gate and not tree_special_entries and
            set(tree_files) == set(actual_tree_files) and
            all(_SHA256_RE.match(digest or "") and
                actual_tree_files.get(path) == digest
                for path, digest in tree_files.items()))
        if not tree_top_gate:
            tree_failures.append("runtime MAVROS source-tree manifest schema/root is invalid")
        if not tree_files_gate:
            tree_failures.append(
                "runtime MAVROS source tree is not the full exact manifested tree")
        direct_tree_gate = (
            tree_files_gate and
            tree_files.get("scripts/mavparam") ==
            source_reports["mavparam_entrypoint"]["sha256"] and
            tree_files.get("src/mavros/param.py") ==
            source_reports["mavros_param_python"]["sha256"] and
            tree_files.get("src/plugins/param.cpp") ==
            source_reports["mavros_param_plugin"]["sha256"] and
            tree_files.get("src/plugins/sys_time.cpp") ==
            source_reports["mavros_timesync_plugin"]["sha256"] and
            tree_files.get("launch/px4_config.yaml") ==
            source_reports["mavros_timesync_config"]["sha256"])
        if not direct_tree_gate:
            tree_failures.append("manifested MAVROS param source identities do not join")
        incomplete.extend(tree_failures)

        build_document = load_json_source("mavros_param_build_manifest")
        build_keys = {
            "schema", "mavros_package_path", "param_plugin_binary_sha256",
            "source_tree_manifest_sha256", "mavparam_entrypoint_sha256",
            "mavparam_python_interpreter_sha256",
            "mavros_param_python_sha256", "mavros_param_runtime_module_sha256",
            "mavros_param_plugin_sha256",
            "mavros_timesync_plugin_sha256", "mavros_timesync_config_sha256",
            "cmake_cache_sha256", "build_marker_sha256"}
        build_schema_gate = (
            isinstance(build_document, dict) and set(build_document) == build_keys and
            build_document.get("schema") ==
            "flight_safety/mavros_param_runtime_build/v1")
        build_join_gate = (
            build_schema_gate and
            os.path.realpath(build_document.get("mavros_package_path", "")) ==
            os.path.realpath(mavros_root or "") and
            build_document.get("param_plugin_binary_sha256") ==
            source_reports["mavros_param_plugin_binary"]["sha256"] and
            build_document.get("source_tree_manifest_sha256") ==
            source_reports["mavros_param_source_tree_manifest"]["sha256"] and
            build_document.get("mavparam_entrypoint_sha256") ==
            source_reports["mavparam_entrypoint"]["sha256"] and
            build_document.get("mavparam_python_interpreter_sha256") ==
            source_reports["mavparam_python_interpreter"]["sha256"] and
            build_document.get("mavros_param_python_sha256") ==
            source_reports["mavros_param_python"]["sha256"] and
            build_document.get("mavros_param_runtime_module_sha256") ==
            source_reports["mavros_param_runtime_module"]["sha256"] and
            build_document.get("mavros_param_plugin_sha256") ==
            source_reports["mavros_param_plugin"]["sha256"] and
            build_document.get("mavros_timesync_plugin_sha256") ==
            source_reports["mavros_timesync_plugin"]["sha256"] and
            build_document.get("mavros_timesync_config_sha256") ==
            source_reports["mavros_timesync_config"]["sha256"] and
            build_document.get("cmake_cache_sha256") ==
            source_reports["mavros_build_cmake_cache"]["sha256"] and
            build_document.get("build_marker_sha256") ==
            source_reports["mavros_build_marker"]["sha256"])
        if not build_join_gate:
            incomplete.append(
                "runtime MAVROS param-plugin binary/build/source manifest join failed")

        acceptance_build = load_json_source(
            "timesync_acceptance_build_manifest")
        acceptance_build_gate = (
            isinstance(acceptance_build, dict) and
            set(acceptance_build) == {
                "schema", "publisher_source_sha256", "publisher_binary_sha256",
                "mavros_timesync_plugin_sha256", "message_schema"} and
            acceptance_build.get("schema") ==
            "flight_safety/timesync_acceptance_runtime_build/v1" and
            acceptance_build.get("message_schema") ==
            TIMESYNC_ACCEPTANCE_SCHEMA and
            acceptance_build.get("publisher_source_sha256") ==
            source_reports["timesync_acceptance_publisher_source"]["sha256"] and
            acceptance_build.get("publisher_binary_sha256") ==
            source_reports["timesync_acceptance_publisher_binary"]["sha256"] and
            acceptance_build.get("mavros_timesync_plugin_sha256") ==
            source_reports["mavros_timesync_plugin"]["sha256"])
        if not acceptance_build_gate:
            incomplete.append(
                "timesync accepted-sequence publisher build/source join failed")

        mavros_topic_publishers = {}
        mavros_topic_graph_gate = True
        for suffix in (
                "/param/param_value", "/state", "/extended_state",
                "/timesync_status", "/estimator_status"):
            topic = self.namespace + suffix
            topic_info = self._capture(
                run, commands, "mavros_publisher_" + suffix.strip("/").replace("/", "_"),
                ["rostopic", "info", topic], self.command_timeout_s)
            publishers = _rostopic_publisher_nodes(topic_info["stdout_text"])
            mavros_topic_publishers[topic] = publishers
            mavros_topic_graph_gate = (
                mavros_topic_graph_gate and topic_info["command_ok"] and
                publishers == [self.namespace])
        if not mavros_topic_graph_gate:
            incomplete.append(
                "FCU state/extended/param/timesync/estimator topics are not "
                "single-publisher-bound "
                "to the runtime MAVROS node")

        mavros_service_providers = {}
        mavros_service_graph_gate = True
        for suffix in ("/vehicle_info_get", "/param/pull"):
            service = self.namespace + suffix
            service_info = self._capture(
                run, commands, "mavros_service_" + suffix.strip("/").replace("/", "_"),
                ["rosservice", "info", service], self.command_timeout_s)
            provider = _rosservice_provider_node(service_info["stdout_text"])
            mavros_service_providers[service] = provider
            mavros_service_graph_gate = (
                mavros_service_graph_gate and service_info["command_ok"] and
                provider == self.namespace)
        if not mavros_service_graph_gate:
            incomplete.append(
                "vehicle-info/parameter-pull services are not bound to runtime MAVROS")

        node_info = self._capture(
            run, commands, "mavros_node_runtime", ["rosnode", "info", self.namespace],
            self.command_timeout_s)
        mavros_node_runtime = _node_runtime_identity(node_info["stdout_text"])
        pid = mavros_node_runtime["pid"]
        mapped_paths = []
        if (node_info["command_ok"] and mavros_node_runtime["local_host_gate"] and
                pid is not None):
            try:
                maps_text = self.process_maps_reader(pid)
                for line in str(maps_text).splitlines():
                    fields = line.split()
                    if fields and fields[-1].startswith("/"):
                        mapped_paths.append(os.path.realpath(fields[-1]))
            except (OSError, TypeError, ValueError):
                mapped_paths = []
        binary_realpath = source_reports["mavros_param_plugin_binary"]["realpath"]
        mapped_binary_gate = (
            node_info["command_ok"] and mavros_node_runtime["local_host_gate"] and
            isinstance(binary_realpath, str) and binary_realpath in mapped_paths)
        if not mapped_binary_gate:
            incomplete.append(
                "local runtime MAVROS process does not map the exact param-plugin binary")

        acceptance_topic_info = self._capture(
            run, commands, "timesync_acceptance_topic_publishers",
            ["rostopic", "info", acceptance_topic], self.command_timeout_s)
        acceptance_publishers = _rostopic_publisher_nodes(
            acceptance_topic_info["stdout_text"])
        acceptance_pid = None
        acceptance_executable = None
        acceptance_node_runtime = {
            "pid": None, "uri": None, "host": None, "local_host_gate": False}
        if acceptance_topic_info["command_ok"] and len(acceptance_publishers) == 1:
            acceptance_node_info = self._capture(
                run, commands, "timesync_acceptance_publisher_node",
                ["rosnode", "info", acceptance_publishers[0]],
                self.command_timeout_s)
            acceptance_node_runtime = _node_runtime_identity(
                acceptance_node_info["stdout_text"])
            acceptance_pid = acceptance_node_runtime["pid"]
            if (acceptance_node_info["command_ok"] and
                    acceptance_node_runtime["local_host_gate"] and
                    acceptance_pid is not None):
                try:
                    acceptance_executable = os.path.realpath(
                        self.process_exe_reader(acceptance_pid))
                except (OSError, TypeError, ValueError):
                    acceptance_executable = None
        configured_acceptance_executable = source_reports[
            "timesync_acceptance_publisher_binary"]["realpath"]
        acceptance_runtime_gate = (
            acceptance_topic_info["command_ok"] and
            len(acceptance_publishers) == 1 and
            acceptance_node_runtime["local_host_gate"] and
            isinstance(acceptance_executable, str) and
            acceptance_executable == configured_acceptance_executable)
        if not acceptance_runtime_gate:
            incomplete.append(
                "accepted-sequence topic is not bound to one exact publisher executable")

        runtime_param_identity = {
            "status": "PASS" if all((
                collector_entrypoint_gate, all(runtime_path_gates.values()),
                module_probe_gate, direct_tree_gate, build_join_gate,
                acceptance_build_gate,
                mavros_topic_graph_gate, mavros_service_graph_gate,
                acceptance_runtime_gate, mapped_binary_gate)) else "FAIL",
            "actual_collector_entrypoint": executed_entrypoint,
            "collector_entrypoint_gate": collector_entrypoint_gate,
            "runtime_mavros_package_path": mavros_root,
            "runtime_source_path_gates": runtime_path_gates,
            "mavparam_runtime_module_probe": module_probe_document,
            "mavparam_runtime_module_probe_gate": module_probe_gate,
            "source_tree_manifest_gate": tree_files_gate,
            "source_tree_symlink_entries": sorted(tree_special_entries),
            "source_tree_direct_join_gate": direct_tree_gate,
            "build_manifest_join_gate": build_join_gate,
            "timesync_acceptance_build_join_gate": acceptance_build_gate,
            "timesync_acceptance_publishers": acceptance_publishers,
            "timesync_acceptance_publisher_pid": acceptance_pid,
            "timesync_acceptance_publisher_node_runtime":
                acceptance_node_runtime,
            "timesync_acceptance_publisher_executable": acceptance_executable,
            "timesync_acceptance_runtime_executable_gate": acceptance_runtime_gate,
            "runtime_mavros_pid": pid,
            "runtime_mavros_node_identity": mavros_node_runtime,
            "mavros_topic_publishers": mavros_topic_publishers,
            "mavros_topic_single_publisher_gate": mavros_topic_graph_gate,
            "mavros_service_providers": mavros_service_providers,
            "mavros_service_provider_gate": mavros_service_graph_gate,
            "mapped_param_plugin_binary_gate": mapped_binary_gate,
            "mapped_library_paths": sorted(set(mapped_paths)),
        }

        source_manifest_paths = config.get("source_manifest_paths", {})
        bundle_report = validate_source_bundle_manifest(
            source_paths.get("bundle_manifest"),
            {
                "fcu_audit_config": {
                    "path": source_manifest_paths.get("fcu_audit_config"),
                    "sha256": source_reports["fcu_audit_config"]["sha256"]},
                "collector_entrypoint": {
                    "path": source_manifest_paths.get("collector_entrypoint"),
                    "sha256": source_reports["collector_entrypoint"]["sha256"]},
                "trace_entrypoint": {
                    "path": source_manifest_paths.get("trace_entrypoint"),
                    "sha256": source_reports["trace_entrypoint"]["sha256"]},
                "timesync_trace_entrypoint": {
                    "path": source_manifest_paths.get("timesync_trace_entrypoint"),
                    "sha256": source_reports[
                        "timesync_trace_entrypoint"]["sha256"]},
                "collector_core": {
                    "path": source_manifest_paths.get("collector_core"),
                    "sha256": core_sources["collector_core"]["sha256"]},
                "append_only_evidence": {
                    "path": source_manifest_paths.get("append_only_evidence"),
                    "sha256": core_sources["append_only_evidence"]["sha256"]},
            },
            {
                label: {
                    "path": source_manifest_paths.get(label),
                    "sha256": source_reports[label]["sha256"]}
                for label in external_labels
            })
        if bundle_report["status"] != "PASS":
            incomplete.extend(bundle_report["failures"])
        identity_gates = (
            [item["gate"] for item in package_reports.values()] +
            [item["gate"] for item in message_reports.values()] +
            [item["gate"] for item in topic_reports.values()] +
            [item["gate"] for item in source_reports.values()] +
            [item["gate"] for item in core_sources.values()] +
            [collector_entrypoint_gate,
             runtime_param_identity["status"] == "PASS",
             bundle_report["status"] == "PASS"])
        return {
            "status": "PASS" if all(identity_gates) else "FAIL",
            "packages": package_reports, "messages": message_reports,
            "runtime_topic_types": topic_reports,
            "supplied_sources": source_reports,
            "collector_sources": core_sources,
            "runtime_param_identity": runtime_param_identity,
            "bundle_manifest_gate": bundle_report,
            "source_identity_scope": (
                "The actual collector/trace/mavparam entrypoints, runtime package "
                "param.py/source tree, mapped param-plugin binary, build inputs, "
                "message MD5s, topic types, and accepted-sequence publisher identity "
                "are exact-bound before any forced pull."),
        }

    def _source_identity_unchanged(self, identity_report):
        failures = []
        source_gates = {}
        for label, record in sorted(
                identity_report.get("supplied_sources", {}).items()):
            path = record.get("path")
            gate = (
                record.get("gate") is True and isinstance(path, str) and
                os.path.isabs(path) and os.path.isfile(path) and
                not os.path.islink(path) and
                sha256_file(path) == record.get("sha256"))
            source_gates[label] = gate
            if not gate:
                failures.append("pre-pull source changed/unavailable: %s" % label)
        core_gates = {}
        for label, record in sorted(
                identity_report.get("collector_sources", {}).items()):
            path = record.get("path")
            gate = (
                record.get("gate") is True and isinstance(path, str) and
                os.path.isfile(path) and not os.path.islink(path) and
                sha256_file(path) == record.get("sha256"))
            core_gates[label] = gate
            if not gate:
                failures.append("pre-pull collector source changed: %s" % label)

        tree_gate = False
        tree_path = self.config["identity"].get("source_paths", {}).get(
            "mavros_param_source_tree_manifest")
        try:
            with open(tree_path, "r") as stream:
                tree = json.load(stream)
            root = tree.get("package_root")
            expected = tree.get("files")
            actual = {}
            if isinstance(root, str) and os.path.isdir(root):
                for parent, dirs, files in os.walk(root):
                    for item in list(dirs):
                        path = os.path.join(parent, item)
                        if os.path.islink(path):
                            relative = os.path.relpath(path, root).replace(os.sep, "/")
                            actual[relative + "@symlink"] = None
                    dirs[:] = sorted(item for item in dirs
                                     if item not in (".git", "__pycache__") and
                                     not os.path.islink(os.path.join(parent, item)))
                    for filename in sorted(files):
                        path = os.path.join(parent, filename)
                        if os.path.islink(path):
                            relative = os.path.relpath(path, root).replace(os.sep, "/")
                            actual[relative + "@symlink"] = None
                            continue
                        if filename.endswith((".pyc", ".pyo")):
                            continue
                        relative = os.path.relpath(path, root).replace(os.sep, "/")
                        actual[relative] = sha256_file(path)
            tree_gate = (
                isinstance(tree, dict) and
                tree.get("schema") == "flight_safety/mavros_param_source_tree/v1" and
                isinstance(expected, dict) and bool(expected) and expected == actual)
        except (OSError, TypeError, ValueError):
            tree_gate = False
        if not tree_gate:
            failures.append("pre-pull full MAVROS source tree changed/unavailable")
        return {
            "status": "PASS" if not failures else "FAIL",
            "failures": failures, "supplied_source_digest_gates": source_gates,
            "collector_source_digest_gates": core_gates,
            "full_mavros_source_tree_gate": tree_gate,
        }

    def _runtime_identity_unchanged(self, run, commands, identity_report, phase):
        """Rejoin ROS graph, local PIDs, loaded plugin, and Python import at pull."""
        failures = []
        prefix = str(phase).strip().replace("/", "_")
        if not prefix:
            raise ValueError("runtime recheck phase is empty")
        initial = identity_report.get("runtime_param_identity", {})
        expected_mavros_pid = initial.get("runtime_mavros_pid")
        expected_acceptance_pid = initial.get(
            "timesync_acceptance_publisher_pid")
        expected_acceptance_publishers = initial.get(
            "timesync_acceptance_publishers")
        source_reports = identity_report.get("supplied_sources", {})

        sim_capture = self._capture(
            run, commands, prefix + "_global_use_sim_time",
            ["rosparam", "get", "/use_sim_time"], self.command_timeout_s)
        try:
            use_sim_time = yaml.safe_load(sim_capture["stdout_text"])
        except yaml.YAMLError:
            use_sim_time = None
        sim_time_gate = sim_capture["command_ok"] and use_sim_time is False
        if not sim_time_gate:
            failures.append("%s global /use_sim_time changed or is unavailable" % prefix)

        topic_publishers = {}
        topic_gate = True
        for suffix in (
                "/param/param_value", "/state", "/extended_state",
                "/timesync_status", "/estimator_status"):
            topic = self.namespace + suffix
            capture = self._capture(
                run, commands,
                prefix + "_publisher_" + suffix.strip("/").replace("/", "_"),
                ["rostopic", "info", topic], self.command_timeout_s)
            publishers = _rostopic_publisher_nodes(capture["stdout_text"])
            topic_publishers[topic] = publishers
            topic_gate = (topic_gate and capture["command_ok"] and
                          publishers == [self.namespace])
        if not topic_gate:
            failures.append(
                "%s FCU topic publisher set changed or is not exact MAVROS" % prefix)

        service_providers = {}
        service_gate = True
        for suffix in ("/vehicle_info_get", "/param/pull"):
            service = self.namespace + suffix
            capture = self._capture(
                run, commands,
                prefix + "_service_" + suffix.strip("/").replace("/", "_"),
                ["rosservice", "info", service], self.command_timeout_s)
            provider = _rosservice_provider_node(capture["stdout_text"])
            service_providers[service] = provider
            service_gate = (service_gate and capture["command_ok"] and
                            provider == self.namespace)
        if not service_gate:
            failures.append("%s MAVROS service provider set changed" % prefix)

        mavros_capture = self._capture(
            run, commands, prefix + "_mavros_node_runtime",
            ["rosnode", "info", self.namespace], self.command_timeout_s)
        mavros_runtime = _node_runtime_identity(mavros_capture["stdout_text"])
        mapped_paths = []
        if (mavros_capture["command_ok"] and mavros_runtime["local_host_gate"] and
                mavros_runtime["pid"] is not None):
            try:
                for line in str(self.process_maps_reader(
                        mavros_runtime["pid"])).splitlines():
                    fields = line.split()
                    if fields and fields[-1].startswith("/"):
                        mapped_paths.append(os.path.realpath(fields[-1]))
            except (OSError, TypeError, ValueError):
                mapped_paths = []
        binary = source_reports.get("mavros_param_plugin_binary", {}).get(
            "realpath")
        mavros_process_gate = (
            mavros_capture["command_ok"] and mavros_runtime["local_host_gate"] and
            mavros_runtime["pid"] == expected_mavros_pid and
            isinstance(binary, str) and binary in mapped_paths)
        if not mavros_process_gate:
            failures.append(
                "%s local MAVROS PID or mapped param-plugin identity changed" % prefix)

        acceptance_topic = self.config["timesync"]["acceptance_topic"]
        acceptance_topic_capture = self._capture(
            run, commands, prefix + "_timesync_acceptance_publishers",
            ["rostopic", "info", acceptance_topic], self.command_timeout_s)
        acceptance_publishers = _rostopic_publisher_nodes(
            acceptance_topic_capture["stdout_text"])
        acceptance_runtime = {
            "pid": None, "uri": None, "host": None, "local_host_gate": False}
        acceptance_executable = None
        if (acceptance_topic_capture["command_ok"] and
                len(acceptance_publishers) == 1):
            acceptance_node_capture = self._capture(
                run, commands, prefix + "_timesync_acceptance_node",
                ["rosnode", "info", acceptance_publishers[0]],
                self.command_timeout_s)
            acceptance_runtime = _node_runtime_identity(
                acceptance_node_capture["stdout_text"])
            if (acceptance_node_capture["command_ok"] and
                    acceptance_runtime["local_host_gate"] and
                    acceptance_runtime["pid"] is not None):
                try:
                    acceptance_executable = os.path.realpath(
                        self.process_exe_reader(acceptance_runtime["pid"]))
                except (OSError, TypeError, ValueError):
                    acceptance_executable = None
        configured_acceptance_executable = source_reports.get(
            "timesync_acceptance_publisher_binary", {}).get("realpath")
        acceptance_gate = (
            acceptance_publishers == expected_acceptance_publishers and
            acceptance_runtime["local_host_gate"] and
            acceptance_runtime["pid"] == expected_acceptance_pid and
            acceptance_executable == configured_acceptance_executable)
        if not acceptance_gate:
            failures.append(
                "%s accepted-sequence publisher PID/executable changed" % prefix)

        interpreter = source_reports.get(
            "mavparam_python_interpreter", {}).get("realpath")
        runtime_module = source_reports.get(
            "mavros_param_runtime_module", {})
        probe_code = (
            "import hashlib,json,mavros.param,os,sys;"
            "p=os.path.realpath(mavros.param.__file__);"
            "h=hashlib.sha256(open(p,'rb').read()).hexdigest();"
            "print(json.dumps({'interpreter':os.path.realpath(sys.executable),"
            "'module_path':p,'module_sha256':h},sort_keys=True))")
        probe_capture = self._capture(
            run, commands, prefix + "_mavparam_python_runtime_module",
            [interpreter or "", "-c", probe_code], self.command_timeout_s)
        try:
            probe = json.loads(probe_capture["stdout_text"])
        except (TypeError, ValueError):
            probe = None
        module_gate = (
            probe_capture["command_ok"] and isinstance(probe, dict) and
            set(probe) == {"interpreter", "module_path", "module_sha256"} and
            os.path.realpath(probe.get("interpreter", "")) == interpreter and
            os.path.realpath(probe.get("module_path", "")) ==
            runtime_module.get("realpath") and
            probe.get("module_sha256") == runtime_module.get("sha256"))
        if not module_gate:
            failures.append(
                "%s mavparam interpreter/imported module identity changed" % prefix)
        return {
            "status": "PASS" if not failures else "FAIL",
            "failures": failures, "topic_publishers": topic_publishers,
            "use_sim_time": use_sim_time, "use_sim_time_gate": sim_time_gate,
            "topic_publisher_gate": topic_gate,
            "service_providers": service_providers,
            "service_provider_gate": service_gate,
            "mavros_node_runtime": mavros_runtime,
            "mavros_pid_and_plugin_gate": mavros_process_gate,
            "acceptance_publishers": acceptance_publishers,
            "acceptance_node_runtime": acceptance_runtime,
            "acceptance_publisher_executable": acceptance_executable,
            "acceptance_publisher_gate": acceptance_gate,
            "mavparam_runtime_module_probe": probe,
            "mavparam_runtime_module_gate": module_gate,
        }

    def _capture_topic(self, run, commands, label, topic, count):
        return self._capture(
            run, commands, label,
            ["rostopic", "echo", "-n", str(int(count)), topic],
            self.command_timeout_s)

    def _capture_trace_topic(self, run, commands, label, role, topic, count):
        trace_path = self.config["identity"].get("source_paths", {}).get(
            "trace_entrypoint", "")
        current_gate = (
            self._trace_identity_authorized and isinstance(trace_path, str) and
            os.path.isabs(trace_path) and os.path.isfile(trace_path) and
            not os.path.islink(trace_path) and
            sha256_file(trace_path) == self._source_hashes.get("trace_entrypoint"))
        if not current_gate:
            argv = [trace_path, "--role", role, "--topic", topic,
                    "--count", str(int(count))]
            capture = self._record_capture(
                run, label, argv,
                _result(error="trace capture skipped: source identity not authorized"),
                self.command_timeout_s)
            commands.append(capture)
            return capture
        return self._capture(
            run, commands, label,
            [trace_path, "--role", role, "--topic", topic,
             "--count", str(int(count)), "--connection-timeout",
             str(float(self.config.get("trace_connection_timeout_s", 10.0)))],
            self.command_timeout_s)

    def _capture_identity_bound_helper(self, run, commands, label, source_label,
                                       argv, timeout_s):
        path = self.config["identity"].get("source_paths", {}).get(
            source_label, "")
        current_gate = (
            self._trace_identity_authorized and isinstance(path, str) and
            os.path.isabs(path) and os.path.isfile(path) and
            not os.path.islink(path) and
            sha256_file(path) == self._source_hashes.get(source_label) and
            bool(argv) and argv[0] == path)
        if not current_gate:
            capture = self._record_capture(
                run, label, argv,
                _result(error="helper skipped: source identity not authorized"),
                timeout_s)
            commands.append(capture)
            return capture
        return self._capture(run, commands, label, argv, timeout_s)

    def _capture(self, run, commands, label, argv, timeout_s):
        started = self.clock_ns()
        started_steady = self.steady_clock_ns()
        result = self.command_runner(list(argv), float(timeout_s))
        finished_steady = self.steady_clock_ns()
        finished = self.clock_ns()
        capture = self._record_capture(
            run, label, argv, result, timeout_s, started, finished,
            started_steady, finished_steady)
        commands.append(capture)
        return capture

    def _record_capture(self, run, label, argv, result, timeout_s,
                        started_wall_ns=None, finished_wall_ns=None,
                        started_steady_ns=None, finished_steady_ns=None):
        stdout = str(result.get("stdout") or "")
        stderr = str(result.get("stderr") or "")
        stdout_artifact = run.write_text("evidence/%s.stdout.txt" % label, stdout)
        stderr_artifact = run.write_text("evidence/%s.stderr.txt" % label, stderr)
        started = int(result.get("started_wall_ns", started_wall_ns or self.clock_ns()))
        finished = int(result.get("finished_wall_ns", finished_wall_ns or self.clock_ns()))
        started_steady = int(result.get(
            "started_steady_ns", started_steady_ns or self.steady_clock_ns()))
        finished_steady = int(result.get(
            "finished_steady_ns", finished_steady_ns or self.steady_clock_ns()))
        return {
            "label": label, "argv": list(argv), "read_only": True,
            "timeout_s": float(timeout_s), "returncode": result.get("returncode"),
            "timed_out": bool(result.get("timed_out", False)),
            "error": result.get("error"),
            "command_ok": (result.get("returncode") == 0 and
                           not result.get("timed_out", False) and
                           not result.get("error")),
            "started_wall_ns": started, "finished_wall_ns": finished,
            "started_steady_ns": started_steady,
            "finished_steady_ns": finished_steady,
            "stdout_artifact": stdout_artifact, "stderr_artifact": stderr_artifact,
            "stdout_text": stdout,
        }

    def _default_pull_transaction(self, spec, timeout_s):
        """Run three raw subscribers across the entire synchronous force pull."""
        started = self.clock_ns()
        started_steady = self.steady_clock_ns()
        trace_keys = (
            ("param_trace", "param_trace_argv", "param"),
            ("state_trace", "state_trace_argv", "state"),
            ("extended_trace", "extended_trace_argv", "extended_state"))
        processes = {}
        files = {}
        results = {}
        alive_until_stop = {}
        ready_keys = set()
        ready_reports = {}
        safety_sample_ready = False
        try:
            for key, argv_key, _role in trace_keys:
                out_file = tempfile.TemporaryFile()
                err_file = tempfile.TemporaryFile()
                files[key] = (out_file, err_file)
                try:
                    processes[key] = subprocess.Popen(
                        spec[argv_key], stdout=out_file, stderr=err_file,
                        env=dict(os.environ, PYTHONUNBUFFERED="1"))
                except OSError as exc:
                    results[key] = _result(
                        None, "", "", False, "%s: %s" % (type(exc).__name__, exc))
            deadline = time.monotonic() + self.trace_startup_s
            while True:
                for key, _argv_key, role in trace_keys:
                    process = processes.get(key)
                    if process is None or process.poll() is not None:
                        continue
                    out_file = files[key][0]
                    try:
                        data = os.pread(out_file.fileno(), 65536, 0).decode(
                            "utf-8", errors="replace")
                    except (AttributeError, OSError):
                        data = ""
                    report = _trace_envelopes(data, role)
                    if (report["status"] == "PASS" and
                            report["ready_handshake_gate"] and
                            (role == "param" or report["sample_count"] >= 1)):
                        ready_keys.add(key)
                        ready_reports[key] = report
                if len(ready_keys) == len(trace_keys) or time.monotonic() >= deadline:
                    break
                time.sleep(0.02)
            dump_started = self.clock_ns()
            dump_started_steady = self.steady_clock_ns()
            first_state = _trace_payloads(
                ready_reports.get("state_trace", {}))
            first_extended = _trace_payloads(
                ready_reports.get("extended_trace", {}))
            state_record = (ready_reports.get("state_trace", {}).get("samples") or
                            [None])[0]
            extended_record = (ready_reports.get(
                "extended_trace", {}).get("samples") or [None])[0]
            max_age_ns = int(float(
                self.config["state_guard"].get("max_age_s", 1.5)) * 1.0e9)
            max_future_ns = int(float(
                self.config["state_guard"].get("max_future_s", 0.05)) * 1.0e9)
            first_headers_fresh = all(
                isinstance(record, dict) and
                -max_future_ns <= (
                    record.get("arrival_ros_ns", 0) -
                    record.get("header_stamp_ns", 0)) <= max_age_ns
                for record in (state_record, extended_record))
            safety_sample_ready = (
                bool(first_state) and bool(first_extended) and
                first_headers_fresh and
                first_state[0].get("connected") is True and
                first_state[0].get("armed") is False and
                first_extended[0].get("landed_state") == LANDED_STATE_ON_GROUND)
            trace_ready = (
                set(processes) == {"param_trace", "state_trace", "extended_trace"} and
                ready_keys == {"param_trace", "state_trace", "extended_trace"} and
                safety_sample_ready and
                all(process.poll() is None for process in processes.values()))
            if trace_ready:
                dump = _default_runner(spec["dump_argv"], timeout_s)
            else:
                dump = _result(
                    None, "", "", False,
                    "force pull skipped: raw/safety trace subscribers not alive")
            dump_finished = self.clock_ns()
            dump_finished_steady = self.steady_clock_ns()
            dump["started_wall_ns"] = dump_started
            dump["finished_wall_ns"] = dump_finished
            dump["started_steady_ns"] = dump_started_steady
            dump["finished_steady_ns"] = dump_finished_steady
            results["dump"] = dump
            if self.trace_tail_s:
                time.sleep(self.trace_tail_s)
        finally:
            for key, process in processes.items():
                alive_until_stop[key] = process.poll() is None
                if alive_until_stop[key]:
                    process.terminate()
            for key, process in processes.items():
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
                out_file, err_file = files[key]
                out_file.seek(0)
                err_file.seek(0)
                stdout = out_file.read().decode("utf-8", errors="replace")
                stderr = err_file.read().decode("utf-8", errors="replace")
                old = results.get(key)
                if old is None:
                    # SIGTERM after the bounded capture window is expected. A
                    # subscriber that exited earlier did not span the pull.
                    if alive_until_stop.get(key):
                        old = _result(0, stdout, stderr)
                    else:
                        old = _result(
                            process.returncode, stdout, stderr, False,
                            "trace subscriber exited before pull window closed")
                else:
                    old["stdout"] = stdout
                    old["stderr"] = stderr
                results[key] = old
                out_file.close()
                err_file.close()
            for key in set(files) - set(processes):
                out_file, err_file = files[key]
                out_file.close()
                err_file.close()
        finished = self.clock_ns()
        finished_steady = self.steady_clock_ns()
        results["started_wall_ns"] = started
        results["finished_wall_ns"] = finished
        results["started_steady_ns"] = started_steady
        results["finished_steady_ns"] = finished_steady
        results["ready_handshake"] = (
            ready_keys == {"param_trace", "state_trace", "extended_trace"})
        results["safety_sample_handshake"] = bool(safety_sample_ready)
        results["concurrent"] = (
            set(processes) == {"param_trace", "state_trace", "extended_trace"} and
            results["ready_handshake"] and results["safety_sample_handshake"] and
            all(alive_until_stop.get(key, False) for key, _, _ in trace_keys) and
            all(results[key].get("returncode") == 0 for key, _, _ in trace_keys))
        return results
