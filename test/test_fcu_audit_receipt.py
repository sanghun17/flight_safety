import hashlib
import json
import os
from pathlib import Path

import pytest
import yaml

from flight_safety.append_only_evidence import self_hash_valid
import flight_safety.append_only_evidence as append_module
import flight_safety.fcu_audit_receipt as collector_module
from flight_safety.fcu_audit_receipt import (
    EXPECTED_MESSAGE_IDENTITIES, FcuAuditCollector, _analyze_param_trace)


ROOT = Path(__file__).resolve().parents[1]
NOW_NS = 1_700_000_100_000_000_000
NOW_STEADY_NS = 9_000_000_000_000
TRACE_SCHEMA = "flight_safety/fcu_trace_envelope/v1"
MANIFEST_PATHS = {
    "fcu_audit_config": "config/fcu_audit_read_only.yaml",
    "collector_entrypoint": "scripts/fcu_audit_receipt.py",
    "trace_entrypoint": "scripts/fcu_trace_capture.py",
    "timesync_trace_entrypoint": "scripts/fcu_timesync_trace_capture.py",
    "collector_core": "src/flight_safety/fcu_audit_receipt.py",
    "append_only_evidence": "src/flight_safety/append_only_evidence.py",
    "mavparam_entrypoint": "external/mavros/scripts/mavparam",
    "mavparam_python_interpreter": "external/python/python3",
    "mavros_param_python": "external/mavros/src/mavros/param.py",
    "mavros_param_runtime_module": "external/python/mavros/param.py",
    "mavros_param_plugin": "external/mavros/src/plugins/param.cpp",
    "mavros_param_plugin_binary": "external/mavros/lib/libmavros_plugins.so",
    "mavros_param_source_tree_manifest": "external/mavros/source_tree.json",
    "mavros_param_build_manifest": "external/mavros/build.json",
    "mavros_build_cmake_cache": "external/mavros/CMakeCache.txt",
    "mavros_build_marker": "external/mavros/.built_by",
    "mavros_timesync_plugin": "external/mavros/sys_time.cpp",
    "mavros_timesync_config": "external/mavros/px4_config.yaml",
    "timesync_acceptance_publisher_source": "external/acceptance/source.cpp",
    "timesync_acceptance_publisher_binary": "external/acceptance/publisher",
    "timesync_acceptance_build_manifest": "external/acceptance/build.json",
}


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _header(stamp_ns):
    return {
        "seq": 0,
        "stamp": {
            "secs": stamp_ns // 1_000_000_000,
            "nsecs": stamp_ns % 1_000_000_000,
        },
        "frame_id": "",
    }


def _documents(items):
    return "\n---\n".join(items) + "\n"


def _state(armed=False, connected=True, stamp_ns=NOW_NS - 10_000_000):
    return {
        "header": _header(stamp_ns), "connected": connected, "armed": armed,
        "guided": False, "manual_input": True, "mode": "MANUAL",
        "system_status": 3,
    }


def _extended(landed=1, stamp_ns=NOW_NS - 10_000_000):
    return {
        "header": _header(stamp_ns), "vtol_state": 0, "landed_state": landed,
    }


def _estimator(stamp_ns, vertical=True):
    values = {
        "header": _header(stamp_ns),
        "attitude_status_flag": True,
        "velocity_horiz_status_flag": True,
        "velocity_vert_status_flag": True,
        "pos_horiz_rel_status_flag": True,
        "pos_horiz_abs_status_flag": False,
        "pos_vert_abs_status_flag": vertical,
        "pos_vert_agl_status_flag": False,
        "const_pos_mode_status_flag": False,
        "pred_pos_horiz_rel_status_flag": True,
        "pred_pos_horiz_abs_status_flag": False,
        "gps_glitch_status_flag": False,
        "accel_error_status_flag": False,
    }
    return yaml.safe_dump(values, sort_keys=True).strip()


def _timesync(stamp_ns, sequence, rtt=2.0):
    return yaml.safe_dump({
        "header": _header(stamp_ns),
        "remote_timestamp_ns": 2_000_000_000 + sequence * 100_000_000,
        "observed_offset_ns": 1000, "estimated_offset_ns": 1000,
        "round_trip_time_ms": rtt,
    }, sort_keys=True).strip()


def _diagnostic(count=600, level=0, stamp_ns=NOW_NS - 10_000_000,
                duplicate_count=False):
    values = [
        {"key": "Timesyncs since startup", "value": str(count)},
        {"key": "Last RTT (ms)", "value": "2.0"},
    ]
    if duplicate_count:
        values.append({"key": "Timesyncs since startup", "value": str(count + 1)})
    return yaml.safe_dump({
        "header": _header(stamp_ns),
        "status": [{
            "level": level, "name": "mavros: Time Sync", "message": "Normal",
            "hardware_id": "mavros", "values": values,
        }],
    }, sort_keys=True).strip()


def _trace(role, payloads, arrival_ros_ns, arrival_steady_ns):
    types = {
        "param": "mavros_msgs/Param", "state": "mavros_msgs/State",
        "extended_state": "mavros_msgs/ExtendedState",
    }
    topics = {
        "param": "/mavros/param/param_value", "state": "/mavros/state",
        "extended_state": "/mavros/extended_state",
    }
    if isinstance(arrival_ros_ns, int):
        arrival_ros_ns = [arrival_ros_ns + index for index in range(len(payloads))]
    if isinstance(arrival_steady_ns, int):
        arrival_steady_ns = [
            arrival_steady_ns + index for index in range(len(payloads))]
    ready_ros = arrival_ros_ns[0] - 20_000_000
    ready_steady = arrival_steady_ns[0] - 20_000_000
    records = [{
        "schema": TRACE_SCHEMA, "kind": "ready", "role": role,
        "topic": topics[role], "message_type": types[role],
        "arrival_ros_ns": ready_ros, "arrival_steady_ns": ready_steady,
        "publisher_connections": 1,
    }]
    for payload, arrival_ros, arrival_steady in zip(
            payloads, arrival_ros_ns, arrival_steady_ns):
        stamp = payload["header"]["stamp"]
        records.append({
            "schema": TRACE_SCHEMA, "kind": "sample", "role": role,
            "topic": topics[role], "message_type": types[role],
            "arrival_ros_ns": arrival_ros,
            "arrival_steady_ns": arrival_steady,
            "header_stamp_ns": stamp["secs"] * 1_000_000_000 + stamp["nsecs"],
            "payload": payload,
        })
    return "\n".join(json.dumps(item, sort_keys=True) for item in records) + "\n"


def _timesync_bundle_trace(timesync_payloads, diagnostic_payloads,
                           acceptance_payloads):
    schema = "flight_safety/fcu_timesync_trace/v1"
    topics = {
        "timesync_status": "/mavros/timesync_status",
        "diagnostics": "/diagnostics",
        "accepted_sequence":
            "/flight_safety/mavros_timesync_accepted_sequence",
    }
    types = {
        "timesync_status": "mavros_msgs/TimesyncStatus",
        "diagnostics": "diagnostic_msgs/DiagnosticArray",
        "accepted_sequence": "diagnostic_msgs/DiagnosticArray",
    }
    pending = []
    stream_offsets = {
        "timesync_status": 0, "accepted_sequence": 1_000_000,
        "diagnostics": 2_000_000}
    for stream, payloads in (
            ("timesync_status", timesync_payloads),
            ("diagnostics", diagnostic_payloads),
            ("accepted_sequence", acceptance_payloads)):
        for payload in payloads:
            stamp = payload["header"]["stamp"]
            header_ns = stamp["secs"] * 1_000_000_000 + stamp["nsecs"]
            arrival_ros = header_ns + 10_000_000 + stream_offsets[stream]
            arrival_steady = NOW_STEADY_NS - (NOW_NS - arrival_ros)
            pending.append((arrival_steady, stream, arrival_ros, header_ns, payload))
    pending.sort(key=lambda item: item[0])
    ready_arrival_steady = pending[0][0] - 20_000_000
    ready_arrival_ros = pending[0][2] - 20_000_000
    records = [{
        "schema": schema, "kind": "ready", "collector_sequence": 0,
        "arrival_ros_ns": ready_arrival_ros,
        "arrival_steady_ns": ready_arrival_steady,
        "streams": {
            stream: {"topic": topics[stream], "message_type": types[stream],
                     "publisher_connections": 1}
            for stream in sorted(topics)},
    }]
    last_steady = ready_arrival_steady
    last_ros = ready_arrival_ros
    for sequence, (arrival_steady, stream, arrival_ros, header_ns, payload) in enumerate(
            pending, 1):
        arrival_steady = max(arrival_steady, last_steady + 1)
        arrival_ros = max(arrival_ros, last_ros)
        records.append({
            "schema": schema, "kind": "sample", "stream": stream,
            "topic": topics[stream], "message_type": types[stream],
            "collector_sequence": sequence, "arrival_ros_ns": arrival_ros,
            "arrival_steady_ns": arrival_steady, "header_stamp_ns": header_ns,
            "payload": payload,
        })
        last_steady = arrival_steady
        last_ros = arrival_ros
    return "\n".join(json.dumps(item, sort_keys=True) for item in records) + "\n"


def _source_files(tmp_path, effective):
    mavros_root = tmp_path / "mavros_runtime"
    (mavros_root / "scripts").mkdir(parents=True)
    (mavros_root / "src/mavros").mkdir(parents=True)
    (mavros_root / "src/plugins").mkdir(parents=True)
    (mavros_root / "launch").mkdir(parents=True)
    files = {
        "collector_entrypoint": str(ROOT / "scripts/fcu_audit_receipt.py"),
        "trace_entrypoint": str(ROOT / "scripts/fcu_trace_capture.py"),
        "timesync_trace_entrypoint": str(
            ROOT / "scripts/fcu_timesync_trace_capture.py"),
    }
    for label, relative in (
            ("mavparam_entrypoint", "scripts/mavparam"),
            ("mavros_param_python", "src/mavros/param.py"),
            ("mavros_param_plugin", "src/plugins/param.cpp")):
        path = mavros_root / relative
        path.write_text(label + "\n")
        if label == "mavparam_entrypoint":
            path.chmod(0o755)
        files[label] = str(path)

    for label in (
            "mavros_param_plugin_binary", "mavros_build_cmake_cache",
            "mavros_build_marker",
            "timesync_acceptance_publisher_source",
            "timesync_acceptance_publisher_binary"):
        path = tmp_path / (label + ".bin")
        path.write_bytes((label + "\n").encode())
        files[label] = str(path)
    timesync_plugin = mavros_root / "src/plugins/sys_time.cpp"
    timesync_plugin.write_text("mavros_timesync_plugin\n")
    files["mavros_timesync_plugin"] = str(timesync_plugin)
    interpreter = tmp_path / "qualified_python3"
    interpreter.write_bytes(b"qualified-python-interpreter\n")
    interpreter.chmod(0o755)
    files["mavparam_python_interpreter"] = str(interpreter)
    runtime_module = tmp_path / "runtime_mavros_param.py"
    runtime_module.write_bytes(Path(files["mavros_param_python"]).read_bytes())
    files["mavros_param_runtime_module"] = str(runtime_module)

    source = {}
    for key, value in effective.items():
        parent = source
        parts = key.split("/")
        for part in parts[:-1]:
            parent = parent.setdefault(part, {})
        parent[parts[-1]] = value
    timesync_config = mavros_root / "launch/px4_config.yaml"
    timesync_config.write_text(yaml.safe_dump(source, sort_keys=True))
    files["mavros_timesync_config"] = str(timesync_config)

    acceptance_build = tmp_path / "timesync_acceptance_build.json"
    acceptance_build.write_text(json.dumps({
        "schema": "flight_safety/timesync_acceptance_runtime_build/v1",
        "message_schema": "flight_safety/mavros_timesync_acceptance/v1",
        "publisher_source_sha256": _sha(
            files["timesync_acceptance_publisher_source"]),
        "publisher_binary_sha256": _sha(
            files["timesync_acceptance_publisher_binary"]),
        "mavros_timesync_plugin_sha256": _sha(files["mavros_timesync_plugin"]),
    }, sort_keys=True))
    files["timesync_acceptance_build_manifest"] = str(acceptance_build)

    tree_files = {}
    for path in sorted(mavros_root.rglob("*")):
        if path.is_file():
            tree_files[str(path.relative_to(mavros_root))] = _sha(path)
    tree_manifest = tmp_path / "mavros_source_tree.json"
    tree_manifest.write_text(json.dumps({
        "schema": "flight_safety/mavros_param_source_tree/v1",
        "package_root": str(mavros_root), "files": tree_files,
    }, sort_keys=True))
    files["mavros_param_source_tree_manifest"] = str(tree_manifest)

    build_manifest = tmp_path / "mavros_build.json"
    build_manifest.write_text(json.dumps({
        "schema": "flight_safety/mavros_param_runtime_build/v1",
        "mavros_package_path": str(mavros_root),
        "param_plugin_binary_sha256": _sha(files["mavros_param_plugin_binary"]),
        "source_tree_manifest_sha256": _sha(tree_manifest),
        "mavparam_entrypoint_sha256": _sha(files["mavparam_entrypoint"]),
        "mavparam_python_interpreter_sha256": _sha(
            files["mavparam_python_interpreter"]),
        "mavros_param_python_sha256": _sha(files["mavros_param_python"]),
        "mavros_param_runtime_module_sha256": _sha(
            files["mavros_param_runtime_module"]),
        "mavros_param_plugin_sha256": _sha(files["mavros_param_plugin"]),
        "mavros_timesync_plugin_sha256": _sha(
            files["mavros_timesync_plugin"]),
        "mavros_timesync_config_sha256": _sha(
            files["mavros_timesync_config"]),
        "cmake_cache_sha256": _sha(files["mavros_build_cmake_cache"]),
        "build_marker_sha256": _sha(files["mavros_build_marker"]),
    }, sort_keys=True))
    files["mavros_param_build_manifest"] = str(build_manifest)

    local_hashes = {
        MANIFEST_PATHS["collector_entrypoint"]: _sha(files["collector_entrypoint"]),
        MANIFEST_PATHS["trace_entrypoint"]: _sha(files["trace_entrypoint"]),
        MANIFEST_PATHS["timesync_trace_entrypoint"]: _sha(
            files["timesync_trace_entrypoint"]),
        MANIFEST_PATHS["collector_core"]: _sha(collector_module.__file__),
        MANIFEST_PATHS["append_only_evidence"]: _sha(append_module.__file__),
    }
    manifest = {
        "schema": "flight_safety/fcu_shadow_source_bundle/v3",
        "files": dict(local_hashes),
        "file_roles": {
            label: {
                "path": MANIFEST_PATHS[label],
                "sha256": local_hashes[MANIFEST_PATHS[label]],
            }
            for label in (
                "collector_entrypoint", "trace_entrypoint",
                "timesync_trace_entrypoint", "collector_core",
                "append_only_evidence")
        },
        "external_sources": {
            label: {"path": MANIFEST_PATHS[label], "sha256": _sha(files[label])}
            for label in MANIFEST_PATHS
            if label not in (
                "fcu_audit_config",
                "collector_entrypoint", "trace_entrypoint",
                "timesync_trace_entrypoint", "collector_core",
                "append_only_evidence")
        },
    }
    manifest_path = tmp_path / "bundle_manifest.json"
    manifest_path.write_text(json.dumps(manifest, sort_keys=True))
    files["bundle_manifest"] = str(manifest_path)
    return files


def _config(tmp_path):
    effective = {
        "conn/timesync_rate": 10.0,
        "time/timesync_mode": "MAVLINK",
        "time/timesync_alpha_initial": 0.05,
        "time/timesync_beta_initial": 0.05,
        "time/timesync_alpha_final": 0.003,
        "time/timesync_beta_final": 0.003,
        "time/convergence_window": 500,
        "time/max_rtt_sample": 10,
        "time/max_deviation_sample": 100,
        "time/max_consecutive_high_rtt": 5,
        "time/max_consecutive_high_deviation": 5,
        "time/publish_sim_time": False,
    }
    source_paths = _source_files(tmp_path, effective)
    config = {
        "_executed_collector_entrypoint": source_paths["collector_entrypoint"],
        "mavros_namespace": "/mavros", "command_timeout_s": 1.0,
        "param_timeout_s": 1.0, "min_param_count": 5,
        "trace_startup_s": 0.01, "trace_tail_s": 0.0,
        "trace_connection_timeout_s": 1.0,
        "state_guard": {
            "max_age_s": 1.5, "max_future_s": 0.05,
            "max_gap_s": 2.0, "during_min_samples": 3,
        },
        "identity": {
            "required_autopilot": 12, "require_vehicle_uid": True,
            "required_mavros_version": "1.17.0",
            "source_paths": source_paths,
            "source_manifest_paths": dict(MANIFEST_PATHS),
        },
        "timesync": {
            "samples": 40, "tail_samples": 20, "diagnostic_samples": 2,
            "diagnostic_name": "mavros: Time Sync",
            "diagnostic_max_age_s": 2.0, "diagnostic_max_gap_s": 2.0,
            "max_age_s": 1.0, "max_gap_s": 0.5,
            "max_abs_residual_ns": 2_000_000,
            "acceptance_topic": "/flight_safety/mavros_timesync_accepted_sequence",
            "acceptance_samples": 20,
            "acceptance_status_name": "flight_safety: MAVROS Time Sync Acceptance",
            "acceptance_max_age_s": 1.0, "acceptance_max_gap_s": 0.5,
            "acceptance_link_max_delay_s": 0.5,
            "acceptance_link_max_future_s": 0.05,
            "max_future_s": 0.05, "connection_timeout_s": 1.0,
            "capture_timeout_s": 5.0, "tail_grace_s": 0.01,
            "effective_config": effective,
        },
        "estimator_status": {
            "samples": 10, "max_age_s": 1.0, "max_gap_s": 1.0,
            "required_true_flags": [
                "attitude_status_flag", "velocity_horiz_status_flag",
                "velocity_vert_status_flag", "pos_horiz_rel_status_flag"],
            "required_false_flags": [
                "const_pos_mode_status_flag", "gps_glitch_status_flag",
                "accel_error_status_flag"],
            "vertical_position_any_true": [
                "pos_vert_abs_status_flag", "pos_vert_agl_status_flag"],
        },
    }
    policy_path = tmp_path / "fcu_audit_read_only.yaml"
    public_config = {key: value for key, value in config.items()
                     if not key.startswith("_")}
    policy_path.write_text(yaml.safe_dump(public_config, sort_keys=True))
    config["_config_source_path"] = str(policy_path)
    manifest_path = Path(source_paths["bundle_manifest"])
    manifest = json.loads(manifest_path.read_text())
    digest = _sha(policy_path)
    manifest["files"][MANIFEST_PATHS["fcu_audit_config"]] = digest
    manifest["file_roles"]["fcu_audit_config"] = {
        "path": MANIFEST_PATHS["fcu_audit_config"], "sha256": digest}
    manifest_path.write_text(json.dumps(manifest, sort_keys=True))
    return config


class SuccessfulFixture(object):
    def __init__(self, config):
        self.config = config
        self.pull_calls = 0
        self.last_pull_spec = None
        self.vertical = True
        self.diagnostic_levels = [0, 0]
        self.diagnostic_counts = [600, 601]
        self.duplicate_diagnostic_key = False
        self.duplicate_acceptance_key = False
        self.acceptance_level = 0
        self.acceptance_remote_offset = 0
        self.prepend_acceptance_rollback = False
        self.acceptance_available = True
        self.param_pseudo_count = 5
        self.reported_param_count = 6

    @property
    def mavros_root(self):
        return str(Path(
            self.config["identity"]["source_paths"]["mavparam_entrypoint"]).parents[1])

    def acceptance_documents(self):
        hashes = {
            key: _sha(self.config["identity"]["source_paths"][key])
            for key in (
                "mavros_timesync_plugin",
                "timesync_acceptance_publisher_source",
                "timesync_acceptance_publisher_binary",
                "timesync_acceptance_build_manifest")
        }
        documents = []
        first_sequence = 19 if self.prepend_acceptance_rollback else 20
        for sequence in range(first_sequence, 40):
            values = {
                "schema": "flight_safety/mavros_timesync_acceptance/v1",
                "source_plugin_sha256": hashes["mavros_timesync_plugin"],
                "publisher_source_sha256":
                    hashes["timesync_acceptance_publisher_source"],
                "publisher_binary_sha256":
                    hashes["timesync_acceptance_publisher_binary"],
                "publisher_build_manifest_sha256":
                    hashes["timesync_acceptance_build_manifest"],
                "session_id": "fixture-session", "reset_counter": "0",
                "observation_sequence": str(600 + sequence),
                "accepted_sequence": str(
                    999 if sequence == 19 else 500 + sequence),
                "accepted": "true", "converged": "true",
                "remote_timestamp_ns": str(
                    2_000_000_000 + sequence * 100_000_000 +
                    self.acceptance_remote_offset),
                "round_trip_time_ms": "2.0", "deviation_ns": "0",
            }
            raw_values = [{"key": key, "value": value}
                          for key, value in sorted(values.items())]
            if self.duplicate_acceptance_key:
                raw_values.append({"key": "accepted", "value": "false"})
            documents.append(yaml.safe_dump({
                "header": _header(
                    NOW_NS - (39 - sequence) * 100_000_000 - 10_000_000),
                "status": [{
                    "level": self.acceptance_level,
                    "name": "flight_safety: MAVROS Time Sync Acceptance",
                    "message": "converged", "hardware_id": "qualified-publisher",
                    "values": raw_values,
                }],
            }, sort_keys=True).strip())
        return _documents(documents)

    def command(self, argv, _timeout_s):
        if len(argv) >= 2 and argv[1] == "-c":
            stdout = json.dumps({
                "interpreter": os.path.realpath(self.config["identity"][
                    "source_paths"]["mavparam_python_interpreter"]),
                "module_path": os.path.realpath(self.config["identity"][
                    "source_paths"]["mavros_param_runtime_module"]),
                "module_sha256": _sha(self.config["identity"]["source_paths"][
                    "mavros_param_runtime_module"]),
            }, sort_keys=True) + "\n"
        elif argv[:2] == ["rosversion", "mavros"]:
            stdout = "1.17.0\n"
        elif argv[:2] == ["rosversion", "mavros_msgs"]:
            stdout = "1.17.0\n"
        elif argv[:2] == ["rospack", "find"]:
            stdout = self.mavros_root + "\n"
        elif argv[:2] == ["rosmsg", "md5"]:
            stdout = EXPECTED_MESSAGE_IDENTITIES[argv[-1]] + "\n"
        elif argv[:2] == ["rostopic", "type"]:
            types = {
                "/mavros/param/param_value": "mavros_msgs/Param",
                "/mavros/timesync_status": "mavros_msgs/TimesyncStatus",
                "/mavros/estimator_status": "mavros_msgs/EstimatorStatus",
                "/mavros/state": "mavros_msgs/State",
                "/mavros/extended_state": "mavros_msgs/ExtendedState",
                "/diagnostics": "diagnostic_msgs/DiagnosticArray",
                "/flight_safety/mavros_timesync_accepted_sequence":
                    "diagnostic_msgs/DiagnosticArray",
            }
            stdout = types[argv[-1]] + "\n"
        elif argv[:2] == ["rostopic", "info"]:
            publisher = ("/qualified_timesync_acceptance"
                         if argv[-1] == self.config["timesync"]["acceptance_topic"]
                         else "/mavros")
            stdout = (
                "Type: fixture\n\nPublishers:\n"
                " * %s (http://localhost:1234/)\n\nSubscribers: None\n" %
                publisher)
        elif argv[:2] == ["rosservice", "info"]:
            stdout = "Node: /mavros\nURI: rosrpc://localhost:2345\n"
        elif argv[:2] == ["rosnode", "info"]:
            pid = 4243 if argv[-1] == "/qualified_timesync_acceptance" else 4242
            stdout = (
                "--------------------------------------------------------------------------------\n"
                "contacting node http://localhost:1234/ ...\nPid: %d\n" % pid)
        elif argv[:2] == ["rosparam", "get"]:
            if argv[-1] == "/use_sim_time":
                stdout = "false\n"
            else:
                key = argv[-1][len("/mavros/"):]
                stdout = yaml.safe_dump(
                    self.config["timesync"]["effective_config"][key])
        elif argv[:2] == ["rosservice", "call"]:
            stdout = (
                "success: true\nvehicles:\n- available_info: 3\n  sysid: 1\n"
                "  compid: 1\n  autopilot: 12\n  type: 2\n  uid: 42\n"
                "  flight_sw_version: 16909060\n"
                "  flight_custom_version: abcdef01\n")
        elif "--role" in argv:
            role = argv[argv.index("--role") + 1]
            count = int(argv[argv.index("--count") + 1])
            arrivals_ros = [NOW_NS - 15_000_000, NOW_NS - 5_000_000]
            arrivals_steady = [
                NOW_STEADY_NS - 15_000_000, NOW_STEADY_NS - 5_000_000]
            if count == 1:
                arrivals_ros = arrivals_ros[-1:]
                arrivals_steady = arrivals_steady[-1:]
            if role == "state":
                payloads = [_state(stamp_ns=value - 1_000_000)
                            for value in arrivals_ros]
            else:
                payloads = [_extended(stamp_ns=value - 1_000_000)
                            for value in arrivals_ros]
            stdout = _trace(role, payloads, arrivals_ros, arrivals_steady)
            return {
                "returncode": 0, "stdout": stdout, "stderr": "",
                "timed_out": False, "error": None,
                "started_wall_ns": NOW_NS - 50_000_000,
                "finished_wall_ns": NOW_NS,
                "started_steady_ns": NOW_STEADY_NS - 50_000_000,
                "finished_steady_ns": NOW_STEADY_NS,
            }
        elif "--timesync-topic" in argv:
            timesync_payloads = [yaml.safe_load(_timesync(
                NOW_NS - (39 - index) * 100_000_000 - 10_000_000,
                index)) for index in range(40)]
            diagnostic_payloads = [yaml.safe_load(_diagnostic(
                self.diagnostic_counts[index], self.diagnostic_levels[index],
                NOW_NS - (1 - index) * 1_000_000_000 - 10_000_000,
                self.duplicate_diagnostic_key)) for index in range(2)]
            acceptance_payloads = list(yaml.safe_load_all(
                self.acceptance_documents())) if self.acceptance_available else []
            stdout = _timesync_bundle_trace(
                timesync_payloads, diagnostic_payloads, acceptance_payloads)
            if not self.acceptance_available:
                return {"returncode": 1, "stdout": stdout,
                        "stderr": "acceptance minimum unavailable",
                        "timed_out": False, "error": None}
        elif argv[-1] == "/mavros/timesync_status":
            stdout = _documents([
                _timesync(
                    NOW_NS - (39 - index) * 100_000_000 - 10_000_000,
                    index) for index in range(40)])
        elif argv[-1] == "/diagnostics":
            stdout = _documents([
                _diagnostic(
                    self.diagnostic_counts[index],
                    self.diagnostic_levels[index],
                    NOW_NS - (1 - index) * 1_000_000_000 - 10_000_000,
                    self.duplicate_diagnostic_key)
                for index in range(2)])
        elif argv[-1] == "/flight_safety/mavros_timesync_accepted_sequence":
            if not self.acceptance_available:
                return {
                    "returncode": 1, "stdout": "", "stderr": "unavailable",
                    "timed_out": False, "error": None}
            stdout = self.acceptance_documents()
        elif argv[-1] == "/mavros/estimator_status":
            stdout = _documents([
                _estimator(
                    NOW_NS - (9 - index) * 100_000_000 - 10_000_000,
                    self.vertical) for index in range(10)])
        else:
            raise AssertionError("unexpected command: %r" % (argv,))
        return {"returncode": 0, "stdout": stdout, "stderr": "",
                "timed_out": False, "error": None}

    def pull(self, spec, _timeout_s):
        self.pull_calls += 1
        self.last_pull_spec = spec
        names = ["SYS_AUTOSTART", "MAV_SYS_ID", "EKF2_EV_CTRL", "P_A", "P_B"]
        values = [4001, 1, 3, 4, 5]
        Path(spec["dump_argv"][-2]).write_text(
            "#NOTE: fixture\n" + "".join(
                "%s,%s\n" % item for item in zip(names, values)))
        arrivals_ros = [
            NOW_NS - value for value in
            (140_000_000, 125_000_000, 110_000_000,
             95_000_000, 80_000_000, 65_000_000)]
        arrivals_steady = [
            NOW_STEADY_NS - value for value in
            (140_000_000, 125_000_000, 110_000_000,
             95_000_000, 80_000_000, 65_000_000)]
        raw = []
        for index, (name, value) in enumerate(zip(names, values)):
            raw.append({
                "header": _header(arrivals_ros[index] - 1_000_000),
                "param_id": name, "value": {"integer": value, "real": 0.0},
                "param_index": index, "param_count": len(names),
            })
        raw.append({
            "header": _header(arrivals_ros[-1] - 1_000_000),
            "param_id": "_HASH_CHECK",
            "value": {"integer": 123, "real": 0.0},
            "param_index": 65535, "param_count": self.param_pseudo_count,
        })
        state_arrival_ros = [
            NOW_NS - 175_000_000, NOW_NS - 100_000_000,
            NOW_NS - 25_000_000]
        state_arrival_steady = [
            NOW_STEADY_NS - 175_000_000, NOW_STEADY_NS - 100_000_000,
            NOW_STEADY_NS - 25_000_000]
        states = [_state(stamp_ns=value - 1_000_000)
                  for value in state_arrival_ros]
        extended = [_extended(stamp_ns=value - 1_000_000)
                    for value in state_arrival_ros]
        common = {"returncode": 0, "stderr": "", "timed_out": False, "error": None}
        return {
            "dump": dict(
                common, stdout="Parameters received: %d\n" %
                self.reported_param_count,
                started_wall_ns=NOW_NS - 150_000_000,
                finished_wall_ns=NOW_NS - 50_000_000,
                started_steady_ns=NOW_STEADY_NS - 150_000_000,
                finished_steady_ns=NOW_STEADY_NS - 50_000_000),
            "param_trace": dict(
                common, stdout=_trace(
                    "param", raw, arrivals_ros, arrivals_steady)),
            "state_trace": dict(
                common, stdout=_trace(
                    "state", states, state_arrival_ros, state_arrival_steady)),
            "extended_trace": dict(
                common, stdout=_trace(
                    "extended_state", extended, state_arrival_ros,
                    state_arrival_steady)),
            "started_wall_ns": NOW_NS - 200_000_000,
            "finished_wall_ns": NOW_NS,
            "started_steady_ns": NOW_STEADY_NS - 200_000_000,
            "finished_steady_ns": NOW_STEADY_NS,
            "ready_handshake": True, "safety_sample_handshake": True,
            "concurrent": True,
        }


def _collector(tmp_path, fixture=None):
    config = _config(tmp_path)
    fixture = fixture or SuccessfulFixture(config)
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    return FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "7f00 r-xp 0 00:00 0 %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary), fixture


def test_complete_receipt_is_self_hashed_and_explicit_acceptance_bound(tmp_path):
    collector, fixture = _collector(tmp_path)
    path, receipt = collector.collect(str(tmp_path / "runs"), "audit-pass")

    assert receipt["status"] == "PASS"
    assert receipt["failures"] == []
    assert receipt["incomplete_evidence"] == []
    assert fixture.pull_calls == 1
    assert fixture.last_pull_spec["dump_argv"][:2] == [
        collector.config["identity"]["source_paths"][
            "mavparam_python_interpreter"],
        collector.config["identity"]["source_paths"]["mavparam_entrypoint"]]
    assert receipt["guards"]["pre"]["state_ready_gate"] is True
    assert receipt["guards"]["during_pull"]["state_covers_dump_start"] is True
    assert receipt["guards"]["during_pull"]["state_covers_dump_finish"] is True
    trace = receipt["observations"]["parameter_trace"]
    assert trace["status"] == "PASS"
    assert trace["header_fresh_at_actual_arrival_gate"] is True
    assert trace["px4_hash_check_exception_allowed"] is True
    timesync = receipt["observations"]["timesync"]
    assert timesync["explicit_accepted_sequence_gate"] is True
    assert timesync["diagnostic_observation_count_used_as_convergence"] is False
    assert timesync["accepted_sequence"]["source_build_identity_gate"] is True
    assert receipt["implementation_identity"]["runtime_param_identity"]["status"] == "PASS"
    assert receipt["safety_invariants"][
        "forced_pull_refreshes_mavros_ros_parameter_cache"] is True
    assert self_hash_valid(receipt)
    assert json.loads(Path(path).read_text()) == receipt
    with pytest.raises(FileExistsError):
        collector.collect(str(tmp_path / "runs"), "audit-pass")


def test_missing_fcu_is_incomplete_and_never_attempts_pull(tmp_path):
    config = _config(tmp_path)
    pull_calls = []

    def failed(_argv, _timeout):
        return {"returncode": 1, "stdout": "", "stderr": "unavailable",
                "timed_out": False, "error": None}

    collector = FcuAuditCollector(
        config, command_runner=failed,
        pull_runner=lambda *_args: pull_calls.append(True),
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS)
    path, receipt = collector.collect(str(tmp_path / "runs"), "audit-no-fcu")
    assert os.path.isfile(path)
    assert receipt["status"] != "PASS"
    assert pull_calls == []
    assert any("pull skipped" in item for item in receipt["incomplete_evidence"])
    assert self_hash_valid(receipt)


def test_unsafe_fresh_precheck_blocks_pull_fail_closed(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def armed_pre(argv, timeout):
        result = original(argv, timeout)
        if "--role" in argv and argv[argv.index("--role") + 1] == "state":
            result["stdout"] = _trace(
                "state", [_state(armed=True)], NOW_NS - 5_000_000,
                NOW_STEADY_NS - 5_000_000)
        return result

    collector.command_runner = armed_pre
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-armed-pre")
    assert receipt["status"] == "FAIL"
    assert fixture.pull_calls == 0
    assert receipt["guards"]["pre"]["disarmed_all"] is False


def test_single_precheck_sample_cannot_satisfy_continuity_or_cadence(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def one_sample(argv, timeout):
        result = original(argv, timeout)
        if "--role" in argv:
            role = argv[argv.index("--role") + 1]
            arrival_ros = NOW_NS - 5_000_000
            arrival_steady = NOW_STEADY_NS - 5_000_000
            payload = (_state(stamp_ns=arrival_ros - 1_000_000)
                       if role == "state" else
                       _extended(stamp_ns=arrival_ros - 1_000_000))
            result["stdout"] = _trace(
                role, [payload], [arrival_ros], [arrival_steady])
        return result

    collector.command_runner = one_sample
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-one-pre")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["guards"]["pre"]["state_samples"] == 1
    assert receipt["guards"]["pre"]["required_samples_per_stream"] == 2


def test_failed_precheck_command_cannot_hide_measured_armed_sample(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command
    first_state = []

    def armed_with_nonzero_rc(argv, timeout):
        result = original(argv, timeout)
        if (not first_state and "--role" in argv and
                argv[argv.index("--role") + 1] == "state"):
            arrivals_ros = [NOW_NS - 15_000_000, NOW_NS - 5_000_000]
            arrivals_steady = [
                NOW_STEADY_NS - 15_000_000, NOW_STEADY_NS - 5_000_000]
            result["stdout"] = _trace(
                "state", [
                    _state(armed=True, stamp_ns=value - 1_000_000)
                    for value in arrivals_ros], arrivals_ros, arrivals_steady)
            result["returncode"] = 1
            first_state.append(True)
        return result

    collector.command_runner = armed_with_nonzero_rc
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-nonzero-armed-precheck")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["incomplete_evidence"]
    assert receipt["guards"]["pre"]["disarmed_all"] is False
    assert any("disarmed" in item for item in receipt["failures"])


def test_missing_extended_stream_cannot_mask_measured_armed_state(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command
    pre_state_done = []
    pre_extended_done = []

    def asymmetric_precheck(argv, timeout):
        result = original(argv, timeout)
        if "--role" not in argv:
            return result
        role = argv[argv.index("--role") + 1]
        if role == "state" and not pre_state_done:
            arrivals_ros = [NOW_NS - 15_000_000, NOW_NS - 5_000_000]
            arrivals_steady = [
                NOW_STEADY_NS - 15_000_000, NOW_STEADY_NS - 5_000_000]
            result["stdout"] = _trace(
                "state", [_state(armed=True, stamp_ns=value - 1_000_000)
                          for value in arrivals_ros], arrivals_ros,
                arrivals_steady)
            pre_state_done.append(True)
        elif role == "extended_state" and not pre_extended_done:
            result["returncode"] = 1
            result["stdout"] = ""
            pre_extended_done.append(True)
        return result

    collector.command_runner = asymmetric_precheck
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-asymmetric-precheck")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["guards"]["pre"]["state_samples"] == 2
    assert receipt["guards"]["pre"]["extended_state_samples"] == 0
    assert any("disarmed" in item for item in receipt["failures"])


def test_use_sim_time_true_blocks_pull_before_header_freshness_is_trusted(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def sim_time(argv, timeout):
        result = original(argv, timeout)
        if argv == ["rosparam", "get", "/use_sim_time"]:
            result["stdout"] = "true\n"
        return result

    collector.command_runner = sim_time
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-sim-time")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["observations"]["global_clock_domain"][
        "exact_false_gate"] is False


def test_runtime_binary_or_manifest_identity_failure_blocks_pull(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "7f00 r-xp 0 00:00 0 /wrong/lib.so\n")
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-runtime-mismatch")
    assert fixture.pull_calls == 0
    assert receipt["implementation_identity"]["runtime_param_identity"][
        "mapped_param_plugin_binary_gate"] is False
    assert receipt["status"] != "PASS"


def test_blank_required_runtime_source_blocks_pull_before_mavparam(tmp_path):
    config = _config(tmp_path)
    config["identity"]["source_paths"]["mavros_param_plugin_binary"] = ""
    fixture = SuccessfulFixture(config)
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "")
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-blank-source")
    assert fixture.pull_calls == 0
    assert receipt["implementation_identity"]["status"] == "FAIL"
    assert any("required source identity is unavailable" in item
               for item in receipt["incomplete_evidence"])


def test_effective_policy_must_equal_exact_manifest_bound_config_file(tmp_path):
    config = _config(tmp_path)
    config["state_guard"]["max_age_s"] = 1.4
    fixture = SuccessfulFixture(config)
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-policy-drift")
    assert fixture.pull_calls == 0
    assert receipt["status"] != "PASS"
    assert receipt["implementation_identity"]["supplied_sources"][
        "fcu_audit_config"]["effective_content_gate"] is False


def test_policy_schema_rejects_unknown_keys_before_collection(tmp_path):
    config = _config(tmp_path)
    config["silently_ignored_policy_typo"] = True
    with pytest.raises(ValueError, match="exact keys"):
        FcuAuditCollector(config)


def test_mavparam_imported_runtime_module_mismatch_blocks_pull(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    original = fixture.command

    def wrong_import(argv, timeout):
        result = original(argv, timeout)
        if len(argv) >= 2 and argv[1] == "-c":
            payload = json.loads(result["stdout"])
            payload["module_path"] = str(tmp_path / "spoofed_param.py")
            result["stdout"] = json.dumps(payload) + "\n"
        return result

    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=wrong_import, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-import-spoof")
    assert fixture.pull_calls == 0
    assert receipt["implementation_identity"]["runtime_param_identity"][
        "mavparam_runtime_module_probe_gate"] is False


def test_timesync_plugin_source_must_be_inside_exact_runtime_mavros_tree(tmp_path):
    config = _config(tmp_path)
    original_plugin = Path(config["identity"]["source_paths"][
        "mavros_timesync_plugin"])
    outside_plugin = tmp_path / "same-bytes-outside-runtime-tree.cpp"
    outside_plugin.write_bytes(original_plugin.read_bytes())
    config["identity"]["source_paths"]["mavros_timesync_plugin"] = str(
        outside_plugin)
    policy_path = Path(config["_config_source_path"])
    policy_path.write_text(yaml.safe_dump(
        {key: value for key, value in config.items() if not key.startswith("_")},
        sort_keys=True))
    manifest_path = Path(config["identity"]["source_paths"]["bundle_manifest"])
    manifest = json.loads(manifest_path.read_text())
    policy_digest = _sha(policy_path)
    manifest["files"][MANIFEST_PATHS["fcu_audit_config"]] = policy_digest
    manifest["file_roles"]["fcu_audit_config"]["sha256"] = policy_digest
    manifest_path.write_text(json.dumps(manifest, sort_keys=True))
    fixture = SuccessfulFixture(config)
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-timesync-source-outside-tree")
    assert fixture.pull_calls == 0
    runtime = receipt["implementation_identity"]["runtime_param_identity"]
    assert runtime["runtime_source_path_gates"]["mavros_timesync_plugin"] is False
    assert runtime["source_tree_direct_join_gate"] is True


def test_duplicate_state_publisher_graph_blocks_pull(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    original = fixture.command

    def spoofed_graph(argv, timeout):
        result = original(argv, timeout)
        if argv[:2] == ["rostopic", "info"] and argv[-1] == "/mavros/state":
            result["stdout"] = (
                "Publishers:\n * /mavros (http://localhost:1/)\n"
                " * /spoof (http://localhost:2/)\n\nSubscribers: None\n")
        return result

    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=spoofed_graph, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-topic-spoof")
    assert fixture.pull_calls == 0
    assert receipt["implementation_identity"]["runtime_param_identity"][
        "mavros_topic_single_publisher_gate"] is False


def test_mavros_process_restart_at_pre_pull_recheck_blocks_pull(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    original = fixture.command
    mavros_node_checks = []

    def restarted_mavros(argv, timeout):
        result = original(argv, timeout)
        if argv[:2] == ["rosnode", "info"] and argv[-1] == "/mavros":
            mavros_node_checks.append(True)
            if len(mavros_node_checks) >= 2:
                result["stdout"] = result["stdout"].replace("Pid: 4242", "Pid: 9999")
        return result

    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=restarted_mavros, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-mavros-restart")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["implementation_identity"]["pre_pull_runtime_recheck"][
        "mavros_pid_and_plugin_gate"] is False


def test_source_tree_toctou_after_safe_trace_is_failure_and_blocks_pull(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command
    changed = []

    def mutate_after_pre_trace(argv, timeout):
        result = original(argv, timeout)
        if (not changed and "--role" in argv and
                argv[argv.index("--role") + 1] == "extended_state"):
            Path(fixture.config["identity"]["source_paths"][
                "mavros_param_python"]).write_text("changed-after-identity\n")
            changed.append(True)
        return result

    collector.command_runner = mutate_after_pre_trace
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-source-toctou")
    assert fixture.pull_calls == 0
    assert receipt["status"] == "FAIL"
    assert receipt["implementation_identity"]["pre_pull_source_recheck"][
        "status"] == "FAIL"
    assert any("pre-pull source changed" in item for item in receipt["failures"])


def test_armed_during_pull_is_fail_closed_even_when_dump_complete(tmp_path):
    collector, fixture = _collector(tmp_path)
    original_pull = fixture.pull

    def armed_during(spec, timeout):
        result = original_pull(spec, timeout)
        arrivals_ros = [
            NOW_NS - 175_000_000, NOW_NS - 100_000_000,
            NOW_NS - 25_000_000]
        arrivals_steady = [
            NOW_STEADY_NS - 175_000_000, NOW_STEADY_NS - 100_000_000,
            NOW_STEADY_NS - 25_000_000]
        result["state_trace"]["stdout"] = _trace(
            "state", [
                _state(stamp_ns=arrivals_ros[0] - 1_000_000),
                _state(stamp_ns=arrivals_ros[1] - 1_000_000),
                _state(armed=True, stamp_ns=arrivals_ros[2] - 1_000_000)],
            arrivals_ros, arrivals_steady)
        return result

    collector.pull_runner = armed_during
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-armed-during")
    assert receipt["status"] == "FAIL"
    assert receipt["guards"]["during_pull"]["disarmed_all"] is False


def test_buffered_headers_cannot_fake_actual_arrival_dump_coverage(tmp_path):
    collector, fixture = _collector(tmp_path)
    original_pull = fixture.pull

    def buffered(spec, timeout):
        result = original_pull(spec, timeout)
        header_stamps = [
            NOW_NS - 175_000_000, NOW_NS - 100_000_000,
            NOW_NS - 25_000_000]
        tail_arrivals_ros = [
            NOW_NS - 40_000_000, NOW_NS - 30_000_000, NOW_NS - 20_000_000]
        tail_arrivals_steady = [
            NOW_STEADY_NS - 40_000_000, NOW_STEADY_NS - 30_000_000,
            NOW_STEADY_NS - 20_000_000]
        result["state_trace"]["stdout"] = _trace(
            "state", [_state(stamp_ns=value) for value in header_stamps],
            tail_arrivals_ros, tail_arrivals_steady)
        result["extended_trace"]["stdout"] = _trace(
            "extended_state", [_extended(stamp_ns=value) for value in header_stamps],
            tail_arrivals_ros, tail_arrivals_steady)
        return result

    collector.pull_runner = buffered
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-buffered")
    guard = receipt["guards"]["during_pull"]
    assert receipt["status"] == "FAIL"
    assert guard["state_covers_dump_start"] is False
    assert guard["extended_state_covers_dump_start"] is False


def test_measured_pull_trace_handshake_violation_is_failure(tmp_path):
    collector, fixture = _collector(tmp_path)
    original_pull = fixture.pull

    def measured_handshake_violation(spec, timeout):
        result = original_pull(spec, timeout)
        result["ready_handshake"] = False
        result["safety_sample_handshake"] = False
        result["concurrent"] = False
        return result

    collector.pull_runner = measured_handshake_violation
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-measured-handshake-violation")
    assert receipt["status"] == "FAIL"
    assert any(item.startswith("pull transaction:")
               for item in receipt["failures"])
    assert not any("trace subscribers were not ready" in item
                   for item in receipt["incomplete_evidence"])


def test_timesync_error_then_ok_and_counter_rollback_fail(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.diagnostic_levels = [2, 0]
    fixture.diagnostic_counts = [700, 600]
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-diag-rollback")
    diagnostic = receipt["observations"]["timesync"]["diagnostic"]
    assert receipt["status"] == "FAIL"
    assert diagnostic["levels"] == [2, 0]
    assert diagnostic["counter_no_rollback_gate"] is False
    assert diagnostic["all_matching_levels_ok"] is False


def test_duplicate_diagnostic_key_is_preserved_and_rejected(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.duplicate_diagnostic_key = True
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-duplicate-kv")
    diagnostic = receipt["observations"]["timesync"]["diagnostic"]
    assert receipt["status"] == "FAIL"
    assert diagnostic["all_diagnostic_status_schema_gate"] is False
    assert any(item["duplicate_value_keys"]
               for item in diagnostic["raw_statuses"])


def test_duplicate_accepted_sequence_key_is_preserved_and_rejected(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.duplicate_acceptance_key = True
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-accepted-dup")
    accepted = receipt["observations"]["timesync"]["accepted_sequence"]
    assert receipt["status"] == "FAIL"
    assert accepted["status"] == "FAIL"
    assert any(item["duplicate_value_keys"] == ["accepted"]
               for item in accepted["raw_statuses"])


def test_error_level_accepted_sequence_status_never_passes(tmp_path):
    collector, fixture = _collector(tmp_path)
    fixture.acceptance_level = 2
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-accepted-error-level")
    accepted = receipt["observations"]["timesync"]["accepted_sequence"]
    assert receipt["status"] == "FAIL"
    assert accepted["status"] == "FAIL"
    assert accepted["all_matching_levels_ok"] is False
    assert accepted["levels"] == [2] * 20


def test_concurrent_acceptance_must_join_exact_status_remote_timestamps(tmp_path):
    collector, fixture = _collector(tmp_path)
    fixture.acceptance_remote_offset = 1
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-accepted-remote-mismatch")
    accepted = receipt["observations"]["timesync"]["accepted_sequence"]
    assert receipt["status"] == "FAIL"
    assert accepted["timesync_sample_linkage_gate"] is False
    assert accepted["timesync_tail_index_gate"] is False


def test_accepted_sequence_rollback_before_tail_is_rejected(tmp_path):
    collector, fixture = _collector(tmp_path)
    fixture.prepend_acceptance_rollback = True
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-accepted-sequence-rollback")
    accepted = receipt["observations"]["timesync"]["accepted_sequence"]
    assert receipt["status"] == "FAIL"
    assert accepted["typed_record_count"] == 21
    assert accepted["accepted_sequence_no_rollback_gate"] is False
    assert accepted["accepted_sequence_transition_gate"] is False


def test_580_rejected_plus_20_healthy_observations_without_typed_acceptance_never_pass(
        tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.diagnostic_counts = [599, 600]
    fixture.acceptance_available = False
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-no-accepted-seq")
    assert receipt["status"] == "INCOMPLETE"
    assert receipt["observations"]["timesync"][
        "explicit_accepted_sequence_gate"] is False
    assert receipt["observations"]["timesync"]["diagnostic"]["ok"] is True


def test_missing_acceptance_does_not_hide_measured_diagnostic_duplicate(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.acceptance_available = False
    fixture.duplicate_diagnostic_key = True
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-missing-acceptance-duplicate-diagnostic")
    assert receipt["status"] == "FAIL"
    assert receipt["incomplete_evidence"]
    assert any("duplicate KeyValue" in item for item in receipt["failures"])


def test_missing_acceptance_does_not_hide_diagnostic_header_rollback(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.acceptance_available = False
    original = fixture.command

    def duplicate_diagnostic_stamp(argv, timeout):
        result = original(argv, timeout)
        if "--timesync-topic" in argv:
            records = [json.loads(line) for line in result["stdout"].splitlines()
                       if line.strip()]
            diagnostics = [item for item in records
                           if item.get("stream") == "diagnostics"]
            diagnostics[1]["payload"]["header"] = diagnostics[0]["payload"][
                "header"]
            diagnostics[1]["header_stamp_ns"] = diagnostics[0]["header_stamp_ns"]
            result["stdout"] = "\n".join(
                json.dumps(item, sort_keys=True) for item in records) + "\n"
        return result

    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=duplicate_diagnostic_stamp,
        pull_runner=fixture.pull, clock_ns=lambda: NOW_NS,
        steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-missing-acceptance-diag-rollback")
    assert receipt["status"] == "FAIL"
    diagnostic = receipt["observations"]["timesync"]["diagnostic"]
    assert diagnostic["header_stamps_monotonic"] is False
    assert any("header monotonicity" in item for item in receipt["failures"])


def test_hash_check_wrong_count_is_measured_failure_not_incomplete(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    fixture.param_pseudo_count = 999
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    acceptance_binary = config["identity"]["source_paths"][
        "timesync_acceptance_publisher_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: acceptance_binary)
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-hash-count")
    assert receipt["status"] == "FAIL"
    assert receipt["observations"]["parameter_trace"]["status"] == "FAIL"
    assert any("pseudo-parameter" in item for item in receipt["failures"])


def test_nonzero_dump_command_cannot_hide_measured_param_trace_violation(tmp_path):
    collector, fixture = _collector(tmp_path)
    fixture.param_pseudo_count = 999
    original_pull = fixture.pull

    def partial_dump(spec, timeout):
        result = original_pull(spec, timeout)
        result["dump"]["returncode"] = 1
        result["dump"]["stderr"] = "partial dump"
        return result

    collector.pull_runner = partial_dump
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-partial-dump-param-violation")
    assert receipt["status"] == "FAIL"
    assert receipt["incomplete_evidence"]
    assert receipt["observations"]["parameter_trace"]["status"] == "FAIL"
    assert any("pseudo-parameter" in item for item in receipt["failures"])


def test_empty_successful_estimator_capture_cannot_pass(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def empty_estimator(argv, timeout):
        result = original(argv, timeout)
        if (argv[:2] == ["rostopic", "echo"] and
                argv[-1] == "/mavros/estimator_status"):
            result["stdout"] = ""
        return result

    collector.command_runner = empty_estimator
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-empty-estimator")
    assert fixture.pull_calls == 1
    assert receipt["status"] == "INCOMPLETE"
    assert receipt["observations"]["estimator_status"]["status"] == "FAIL"
    assert receipt["observations"]["estimator_status"]["sample_count"] == 0
    assert any("estimator_status continuous contract" in item
               for item in receipt["incomplete_evidence"])


def test_empty_successful_postcheck_streams_cannot_pass(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command
    role_counts = {"state": 0, "extended_state": 0}

    def empty_postcheck(argv, timeout):
        result = original(argv, timeout)
        if "--role" in argv:
            role = argv[argv.index("--role") + 1]
            role_counts[role] += 1
            if role_counts[role] == 2:
                result["stdout"] = ""
        return result

    collector.command_runner = empty_postcheck
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-empty-postcheck")
    assert fixture.pull_calls == 1
    assert receipt["status"] == "INCOMPLETE"
    assert receipt["guards"]["post"]["state_samples"] == 0
    assert receipt["guards"]["post"]["extended_state_samples"] == 0
    assert any("fresh FCU postcheck evidence is missing" in item
               for item in receipt["incomplete_evidence"])


def test_nonempty_malformed_postcheck_trace_is_failure(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command
    state_calls = []

    def malformed_post_state(argv, timeout):
        result = original(argv, timeout)
        if ("--role" in argv and
                argv[argv.index("--role") + 1] == "state"):
            state_calls.append(True)
            if len(state_calls) == 2:
                result["stdout"] = "not-json\n"
        return result

    collector.command_runner = malformed_post_state
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-malformed-postcheck")
    assert fixture.pull_calls == 1
    assert receipt["status"] == "FAIL"
    assert receipt["incomplete_evidence"]
    assert any("postcheck state trace: trace line 1 is not JSON" in item
               for item in receipt["failures"])


def test_acceptance_runtime_publisher_executable_mismatch_blocks_pull(tmp_path):
    config = _config(tmp_path)
    fixture = SuccessfulFixture(config)
    binary = config["identity"]["source_paths"]["mavros_param_plugin_binary"]
    collector = FcuAuditCollector(
        config, command_runner=fixture.command, pull_runner=fixture.pull,
        clock_ns=lambda: NOW_NS, steady_clock_ns=lambda: NOW_STEADY_NS,
        process_maps_reader=lambda _pid: "x %s\n" % binary,
        process_exe_reader=lambda _pid: str(tmp_path / "wrong-publisher"))
    _path, receipt = collector.collect(
        str(tmp_path / "runs"), "audit-acceptance-runtime-mismatch")
    runtime = receipt["implementation_identity"]["runtime_param_identity"]
    assert fixture.pull_calls == 0
    assert receipt["status"] != "PASS"
    assert runtime["timesync_acceptance_runtime_executable_gate"] is False
    assert runtime["timesync_acceptance_publisher_executable"] != config[
        "identity"]["source_paths"]["timesync_acceptance_publisher_binary"]


def test_hash_check_exception_requires_px4_and_exact_n_or_n_plus_one():
    payloads = []
    arrivals_ros = [NOW_NS + index * 10 for index in range(3)]
    arrivals_steady = [NOW_STEADY_NS + index * 10 for index in range(3)]
    for index, name in enumerate(("A", "B")):
        payloads.append({
            "header": _header(arrivals_ros[index]),
            "param_id": name, "value": {"integer": index + 1, "real": 0.0},
            "param_index": index, "param_count": 2,
        })
    payloads.append({
        "header": _header(arrivals_ros[2]),
        "param_id": "_HASH_CHECK", "value": {"integer": 9, "real": 0.0},
        "param_index": 65535, "param_count": 2,
    })
    trace = collector_module._trace_envelopes(
        _trace("param", payloads, arrivals_ros, arrivals_steady), "param")
    kwargs = {
        "timing_config": {"max_age_s": 1.0, "max_future_s": 0.05},
        "ready_record": trace["ready_record"],
        "dump_start_steady_ns": NOW_STEADY_NS - 1,
        "dump_end_steady_ns": NOW_STEADY_NS + 100,
    }
    px4 = _analyze_param_trace(
        trace["samples"], {"A": 1, "B": 2}, 3, 12, 2, **kwargs)
    non_px4 = _analyze_param_trace(
        trace["samples"], {"A": 1, "B": 2}, 3, 3, 2, **kwargs)
    assert px4["status"] == "PASS"
    assert non_px4["status"] == "FAIL"


def test_live_timesync_config_mismatch_is_failure(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def mismatch(argv, timeout):
        result = original(argv, timeout)
        if argv[:2] == ["rosparam", "get"] and argv[-1].endswith(
                "/time/max_rtt_sample"):
            result["stdout"] = "999\n"
        return result

    collector.command_runner = mismatch
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-config-mismatch")
    assert receipt["status"] == "FAIL"
    assert any("effective MAVROS timesync config mismatch" in item
               for item in receipt["failures"])


def test_vehicle_bool_numeric_field_is_rejected_and_pull_skipped(tmp_path):
    collector, fixture = _collector(tmp_path)
    original = fixture.command

    def bool_sysid(argv, timeout):
        result = original(argv, timeout)
        if argv[:2] == ["rosservice", "call"]:
            result["stdout"] = result["stdout"].replace("sysid: 1", "sysid: true")
        return result

    collector.command_runner = bool_sysid
    _path, receipt = collector.collect(str(tmp_path / "runs"), "audit-bool-sysid")
    assert receipt["status"] != "PASS"
    assert fixture.pull_calls == 0


def test_relative_output_root_is_refused(tmp_path, monkeypatch):
    collector, _fixture = _collector(tmp_path)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match="absolute"):
        collector.collect("relative", "audit-relative")


def test_default_transaction_never_pulls_without_ready_handshake(
        tmp_path, monkeypatch):
    collector, _fixture = _collector(tmp_path)
    dump_calls = []

    def spawn_failure(*_args, **_kwargs):
        raise OSError("subscriber unavailable")

    def forbidden_dump(*_args, **_kwargs):
        dump_calls.append(True)
        raise AssertionError("force pull must not execute")

    monkeypatch.setattr(collector_module.subprocess, "Popen", spawn_failure)
    monkeypatch.setattr(collector_module, "_default_runner", forbidden_dump)
    spec = {
        "dump_argv": ["/bound/mavparam", "dump", "unused", "-f"],
        "param_trace_argv": ["/bound/trace", "--role", "param"],
        "state_trace_argv": ["/bound/trace", "--role", "state"],
        "extended_trace_argv": ["/bound/trace", "--role", "extended_state"],
    }
    result = collector._default_pull_transaction(spec, 1.0)
    assert dump_calls == []
    assert result["dump"]["returncode"] is None
    assert result["ready_handshake"] is False
    assert result["safety_sample_handshake"] is False
    assert result["concurrent"] is False


def test_alive_trace_processes_without_first_ready_record_never_pull(
        tmp_path, monkeypatch):
    collector, _fixture = _collector(tmp_path)
    dump_calls = []

    class AliveWithoutReady(object):
        def __init__(self):
            self.returncode = None
            self.stopped = False

        def poll(self):
            return self.returncode if self.stopped else None

        def terminate(self):
            self.stopped = True
            self.returncode = 0

        def wait(self, timeout=None):
            del timeout
            return self.returncode

        def kill(self):
            self.terminate()

    monkeypatch.setattr(
        collector_module.subprocess, "Popen",
        lambda *_args, **_kwargs: AliveWithoutReady())
    monkeypatch.setattr(
        collector_module, "_default_runner",
        lambda *_args, **_kwargs: dump_calls.append(True))
    spec = {
        "dump_argv": ["/bound/mavparam", "dump", "unused", "-f"],
        "param_trace_argv": ["/bound/trace", "--role", "param"],
        "state_trace_argv": ["/bound/trace", "--role", "state"],
        "extended_trace_argv": ["/bound/trace", "--role", "extended_state"],
    }
    result = collector._default_pull_transaction(spec, 1.0)
    assert dump_calls == []
    assert result["ready_handshake"] is False
    assert result["safety_sample_handshake"] is False
    assert result["concurrent"] is False
