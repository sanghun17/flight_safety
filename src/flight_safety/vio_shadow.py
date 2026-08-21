"""Pure, fail-closed metrics for a subscriber-only VIO shadow run."""

from __future__ import print_function

import hashlib
import json
import math
import os
import statistics


SCHEMA = "flight_safety/vio_shadow_preflight_receipt/v3"

PROFILE_KEYS = {
    "safety_invariants", "streams", "identity", "linkage", "hybrid_imu",
    "producer_provenance", "evidence"}
SAFETY_KEYS = {"mode", "allow_ros_publish", "allow_fcu_actuation", "fcu_output_topics"}
STREAM_KEYS = {
    "name", "topic", "message_type", "queue_size", "expected_frame_id",
    "expected_child_frame_id", "min_samples", "min_rate_hz", "max_gap_s",
    "max_idle_s", "max_abs_header_latency_s", "max_first_sample_delay_s",
    "min_window_coverage_ratio", "covariance_symmetry_tolerance",
    "covariance_psd_tolerance", "covariance_min_diagonal",
    "covariance_min_eigenvalue",
    "covariance_max_diagonal", "covariance_max_abs", "require_pose_covariance",
    "require_twist_covariance", "require_pose_payload", "require_twist_payload",
    "require_imu_payload", "require_imu_orientation", "require_imu_covariance",
    "quaternion_norm_tolerance"}
IDENTITY_KEYS = {
    "status_topic", "status_message_type", "status_name", "status_schema",
    "min_samples", "max_idle_s", "max_abs_header_latency_s"}
LINKAGE_KEYS = {
    "correction_stream", "propagated_stream", "max_header_delta_s",
    "max_status_arrival_delta_s", "max_identity_correction_stamp_delta_s",
    "require_typed_identity_join"}
EVIDENCE_KEYS = {
    "require_source_identity", "source_paths", "source_manifest_paths",
    "max_duration_overrun_s"}
EVIDENCE_SOURCE_KEYS = {
    "shadow_entrypoint", "bundle_manifest", "hybrid_imu_node",
    "hybrid_imu_core", "hybrid_imu_launch", "hybrid_imu_build_manifest",
    "clock_domain_publisher_executable", "clock_domain_publisher_source",
    "clock_domain_publisher_build_manifest", "mavros_timesync_plugin",
    "selected_candidate_overlay", "fastlivo_base_config",
    "fastlivo_camera_config", "fastlivo_build_manifest"}
EVIDENCE_MANIFEST_PATH_KEYS = EVIDENCE_SOURCE_KEYS - {"bundle_manifest"}
HYBRID_IMU_KEYS = {
    "output_stream", "ready_topic", "diagnostics_topic", "diagnostic_name",
    "diagnostic_max_gap_s", "diagnostic_max_idle_s", "mapping_start_topic",
    "mapping_start_message_type", "require_mapping_start_after_ready",
    "ready_stable_before_mapping_s", "ready_diagnostic_samples_before_mapping",
    "required_fault_counter_keys",
    "timesync_domain_topic", "timesync_domain_message_type",
    "timesync_domain_status_name", "timesync_domain_schema",
    "required_timesync_domain", "required_d435_clock_source",
    "required_fcu_clock_source", "timesync_source_roles",
    "timesync_measurement_max_age_s", "timesync_max_abs_offset_s",
    "require_timesync_domain"}
PROVENANCE_KEYS = {
    "runtime_param_namespace", "runtime_camera_param_namespace",
    "executable_path", "dynamic_library_paths", "source_root",
    "required_build_manifest_schema",
    "require_exact_effective_params", "effective_param_ephemeral_allowlist",
    "private_camera_param_ephemeral_allowlist"}
FASTLIVO_LIBRARIES = {
    "libimu_proc.so", "liblaser_mapping.so", "liblio.so", "libpre.so", "libvio.so"}
DOMAIN_SOURCE_FIELDS = {
    "hybrid_imu_node_sha256", "hybrid_imu_core_sha256",
    "d435_launch_sha256", "mavros_timesync_plugin_sha256",
    "publisher_executable_sha256", "publisher_source_sha256",
    "publisher_build_manifest_sha256"}
DOMAIN_REQUIRED_SOURCE_ROLES = {
    "hybrid_imu_node_sha256": "hybrid_imu_node",
    "hybrid_imu_core_sha256": "hybrid_imu_core",
    "d435_launch_sha256": "hybrid_imu_launch",
    "mavros_timesync_plugin_sha256": "mavros_timesync_plugin",
    "publisher_executable_sha256": "clock_domain_publisher_executable",
    "publisher_source_sha256": "clock_domain_publisher_source",
    "publisher_build_manifest_sha256": "clock_domain_publisher_build_manifest"}
PRODUCER_IDENTITY_VALUE_KEYS = {
    "selected_candidate_overlay_sha256", "fastlivo_base_config_sha256",
    "fastlivo_camera_config_sha256", "fastlivo_build_manifest_sha256",
    "effective_params_sha256", "private_camera_params_sha256",
    "executable_sha256", "source_tree_sha256", "libimu_proc_sha256",
    "liblaser_mapping_sha256", "liblio_sha256", "libpre_sha256",
    "libvio_sha256"}
IDENTITY_VALUE_KEYS = {
    "schema", "session_id", "reset_counter", "correction_sequence",
    "correction_stamp_ns"} | PRODUCER_IDENTITY_VALUE_KEYS
DOMAIN_VALUE_KEYS = {
    "schema", "session_id", "reset_counter", "measurement_sequence",
    "domain_id", "measurement_domain", "d435_clock_source", "fcu_clock_source",
    "d435_stamp_ns", "fcu_stamp_ns", "measured_offset_ns"} | DOMAIN_SOURCE_FIELDS
MESSAGE_TYPES = {
    "geometry_msgs/PoseStamped", "geometry_msgs/PoseWithCovarianceStamped",
    "nav_msgs/Odometry", "sensor_msgs/Imu"}
STATUS_MESSAGE_TYPES = {"std_msgs/UInt32", "std_msgs/String"}


def _seconds(delta_ns):
    return float(delta_ns) / 1.0e9


def _strict_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(float(value)))


def _finite_positive(value):
    return _strict_number(value) and float(value) > 0.0


def _normalize_key_values(values):
    """Preserve DiagnosticStatus KeyValue order and reject duplicate keys."""
    if isinstance(values, dict):
        pairs = list(values.items())
    elif isinstance(values, (list, tuple)):
        pairs = list(values)
    else:
        return [], {}, [], False
    raw = []
    normalized = {}
    duplicates = []
    well_formed = True
    for item in pairs:
        if isinstance(item, dict) and set(item) == {"key", "value"}:
            key, value = item["key"], item["value"]
        elif isinstance(item, (list, tuple)) and len(item) == 2:
            key, value = item
        else:
            well_formed = False
            continue
        key = str(key)
        value = str(value)
        raw.append({"key": key, "value": value})
        if key in normalized:
            duplicates.append(key)
        normalized[key] = value
    return raw, normalized, sorted(set(duplicates)), well_formed


def _valid_allowlist(value):
    return (
        isinstance(value, list) and all(isinstance(path, str) for path in value) and
        len(value) == len(set(value)) and
        all(path.startswith("/") and path != "/" and
            "//" not in path and "~" not in path and "*" not in path
            for path in value))


def normal_duration_completion(reason, actual_s, requested_s, max_overrun_s):
    return (
        reason == "duration_complete" and _strict_number(actual_s) and
        _finite_positive(requested_s) and _finite_positive(max_overrun_s) and
        float(actual_s) >= float(requested_s) and
        float(actual_s) <= float(requested_s) + float(max_overrun_s))


def _exact_keys(value, expected, label):
    if not isinstance(value, dict):
        raise ValueError("%s must be a mapping" % label)
    unknown = sorted(set(value) - set(expected))
    missing = sorted(set(expected) - set(value))
    if unknown or missing:
        details = []
        if unknown:
            details.append("unknown keys: %s" % ", ".join(unknown))
        if missing:
            details.append("missing keys: %s" % ", ".join(missing))
        raise ValueError("%s has %s" % (label, "; ".join(details)))


def validate_profile(profile, duration_s=None, output_root=None):
    """Reject unknown keys and unsafe/underspecified shadow contracts."""
    _exact_keys(profile, PROFILE_KEYS, "profile")
    safety = profile.get("safety_invariants")
    _exact_keys(safety, SAFETY_KEYS, "safety_invariants")
    if safety.get("mode") != "shadow_only":
        raise ValueError("profile mode must be shadow_only")
    if safety.get("allow_ros_publish") is not False:
        raise ValueError("allow_ros_publish must be false")
    if safety.get("allow_fcu_actuation") is not False:
        raise ValueError("allow_fcu_actuation must be false")
    if safety.get("fcu_output_topics") != []:
        raise ValueError("fcu_output_topics must be an empty list")
    if duration_s is not None and not _finite_positive(duration_s):
        raise ValueError("duration_s must be finite and positive")
    if output_root is not None:
        import os
        if not os.path.isabs(str(output_root)):
            raise ValueError("output_root must be absolute")

    streams = profile.get("streams")
    if not isinstance(streams, list) or not streams:
        raise ValueError("at least one stream is required")
    names = []
    for index, stream in enumerate(streams):
        _exact_keys(stream, STREAM_KEYS, "streams[%d]" % index)
        name = stream.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("stream names must be nonempty strings")
        names.append(name)
        topic = stream.get("topic")
        if (not isinstance(topic, str) or not topic.startswith("/") or
                topic.startswith("/mavros/setpoint")):
            raise ValueError("stream topic must be an absolute non-setpoint topic")
        if stream.get("message_type") not in MESSAGE_TYPES:
            raise ValueError("unsupported stream message_type")
        if not isinstance(stream.get("expected_frame_id"), str) or not stream["expected_frame_id"]:
            raise ValueError("expected_frame_id must be explicit")
        if not isinstance(stream.get("expected_child_frame_id", ""), str):
            raise ValueError("expected_child_frame_id must be a string")
        if (not isinstance(stream.get("queue_size"), int) or
                isinstance(stream.get("queue_size"), bool) or stream["queue_size"] <= 0):
            raise ValueError("queue_size must be a positive integer")
        if (not isinstance(stream.get("min_samples"), int) or
                isinstance(stream.get("min_samples"), bool) or stream["min_samples"] < 2):
            raise ValueError("min_samples must be at least two")
        for key in ("min_rate_hz", "max_gap_s", "max_idle_s",
                    "max_abs_header_latency_s", "max_first_sample_delay_s",
                    "covariance_max_diagonal", "covariance_max_abs",
                    "quaternion_norm_tolerance"):
            if not _finite_positive(stream.get(key)):
                raise ValueError("%s must be finite and positive" % key)
        coverage = stream.get("min_window_coverage_ratio")
        if not _strict_number(coverage) or not 0.0 < float(coverage) <= 1.0:
            raise ValueError("min_window_coverage_ratio must be in (0,1]")
        for key in ("covariance_symmetry_tolerance", "covariance_psd_tolerance"):
            value = stream.get(key)
            if not _strict_number(value) or float(value) < 0.0:
                raise ValueError("%s must be finite and nonnegative" % key)
        for key in ("covariance_min_diagonal", "covariance_min_eigenvalue"):
            if not _finite_positive(stream.get(key)):
                raise ValueError("%s must be finite and strictly positive" % key)
        if float(stream["covariance_min_diagonal"]) > float(stream["covariance_max_diagonal"]):
            raise ValueError("covariance diagonal bounds are inverted")
        for key in ("require_pose_covariance", "require_twist_covariance",
                    "require_pose_payload", "require_twist_payload",
                    "require_imu_payload", "require_imu_orientation",
                    "require_imu_covariance"):
            if not isinstance(stream.get(key), bool):
                raise ValueError("%s must be an explicit boolean" % key)
    if len(names) != len(set(names)):
        raise ValueError("stream names must be unique")

    identity = profile.get("identity")
    _exact_keys(identity, IDENTITY_KEYS, "identity")
    topic = identity.get("status_topic")
    if not isinstance(topic, str) or (topic and not topic.startswith("/")):
        raise ValueError("identity status_topic must be empty or absolute")
    if identity.get("status_message_type") != "diagnostic_msgs/DiagnosticArray":
        raise ValueError("identity join must use diagnostic_msgs/DiagnosticArray")
    for key in ("status_name", "status_schema"):
        if not isinstance(identity.get(key), str) or not identity[key]:
            raise ValueError("identity %s must be explicit" % key)
    if (not isinstance(identity.get("min_samples"), int) or
            isinstance(identity.get("min_samples"), bool) or identity["min_samples"] < 2):
        raise ValueError("identity min_samples must be at least two")
    for key in ("max_idle_s", "max_abs_header_latency_s"):
        if not _finite_positive(identity.get(key)):
            raise ValueError("identity %s must be finite and positive" % key)

    linkage = profile.get("linkage")
    _exact_keys(linkage, LINKAGE_KEYS, "linkage")
    if linkage.get("correction_stream") not in names or linkage.get("propagated_stream") not in names:
        raise ValueError("linkage stream names must reference configured streams")
    if linkage["correction_stream"] == linkage["propagated_stream"]:
        raise ValueError("correction and propagated streams must differ")
    for key in ("max_header_delta_s", "max_status_arrival_delta_s",
                "max_identity_correction_stamp_delta_s"):
        if not _finite_positive(linkage.get(key)):
            raise ValueError("%s must be finite and positive" % key)
    if linkage.get("require_typed_identity_join") is not True:
        raise ValueError("typed identity join must be explicitly required")

    hybrid = profile.get("hybrid_imu")
    _exact_keys(hybrid, HYBRID_IMU_KEYS, "hybrid_imu")
    if hybrid.get("output_stream") not in names:
        raise ValueError("hybrid_imu output_stream must reference a configured stream")
    output_contract = next(item for item in streams
                           if item["name"] == hybrid["output_stream"])
    if output_contract["message_type"] != "sensor_msgs/Imu":
        raise ValueError("hybrid_imu output_stream must use sensor_msgs/Imu")
    for key in ("ready_topic", "diagnostics_topic"):
        topic = hybrid.get(key)
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError("hybrid_imu %s must be absolute" % key)
    if not isinstance(hybrid.get("diagnostic_name"), str) or not hybrid["diagnostic_name"]:
        raise ValueError("hybrid_imu diagnostic_name must be explicit")
    for key in ("diagnostic_max_gap_s", "diagnostic_max_idle_s",
                "ready_stable_before_mapping_s", "timesync_measurement_max_age_s",
                "timesync_max_abs_offset_s"):
        if not _finite_positive(hybrid.get(key)):
            raise ValueError("hybrid_imu %s must be finite and positive" % key)
    for topic_key in ("mapping_start_topic", "timesync_domain_topic"):
        topic = hybrid.get(topic_key)
        if not isinstance(topic, str) or (topic and not topic.startswith("/")):
            raise ValueError("hybrid_imu %s must be empty or absolute" % topic_key)
    if hybrid.get("mapping_start_message_type") != "std_msgs/String":
        raise ValueError("hybrid mapping start must carry a nonempty session String")
    if hybrid.get("timesync_domain_message_type") != "diagnostic_msgs/DiagnosticArray":
        raise ValueError("hybrid timesync domain evidence must be DiagnosticArray")
    for key in ("require_mapping_start_after_ready", "require_timesync_domain"):
        if not isinstance(hybrid.get(key), bool):
            raise ValueError("hybrid_imu %s must be an explicit boolean" % key)
    if (hybrid["require_timesync_domain"] and
            (not isinstance(hybrid.get("required_timesync_domain"), str) or
             not hybrid["required_timesync_domain"])):
        raise ValueError("required_timesync_domain must be explicit")
    if (not isinstance(hybrid.get("ready_diagnostic_samples_before_mapping"), int) or
            isinstance(hybrid.get("ready_diagnostic_samples_before_mapping"), bool) or
            hybrid["ready_diagnostic_samples_before_mapping"] < 2):
        raise ValueError("ready diagnostic samples before mapping must be at least two")
    fault_keys = hybrid.get("required_fault_counter_keys")
    if (not isinstance(fault_keys, list) or not fault_keys or
            any(not isinstance(item, str) or not item for item in fault_keys) or
            len(fault_keys) != len(set(fault_keys))):
        raise ValueError("required fault counter keys must be unique nonempty strings")
    for key in ("timesync_domain_status_name", "timesync_domain_schema",
                "required_d435_clock_source", "required_fcu_clock_source"):
        if not isinstance(hybrid.get(key), str) or not hybrid[key]:
            raise ValueError("hybrid_imu %s must be explicit" % key)
    source_roles = hybrid.get("timesync_source_roles")
    _exact_keys(source_roles, DOMAIN_SOURCE_FIELDS, "hybrid_imu.timesync_source_roles")
    if source_roles != DOMAIN_REQUIRED_SOURCE_ROLES:
        raise ValueError(
            "timesync source roles must bind the exact publisher/plugin source roles")

    provenance = profile.get("producer_provenance")
    _exact_keys(provenance, PROVENANCE_KEYS, "producer_provenance")
    namespace = provenance.get("runtime_param_namespace")
    if not isinstance(namespace, str) or (namespace and not namespace.startswith("/")):
        raise ValueError("runtime_param_namespace must be empty or absolute")
    camera_namespace = provenance.get("runtime_camera_param_namespace")
    if camera_namespace not in ("", "/laserMapping"):
        raise ValueError("runtime camera params must use /laserMapping private namespace")
    for key in ("executable_path", "source_root"):
        if not isinstance(provenance.get(key), str):
            raise ValueError("producer_provenance %s must be a string" % key)
    libraries = provenance.get("dynamic_library_paths")
    _exact_keys(libraries, FASTLIVO_LIBRARIES,
                "producer_provenance.dynamic_library_paths")
    if any(not isinstance(value, str) for value in libraries.values()):
        raise ValueError("dynamic library paths must be strings")
    if (not isinstance(provenance.get("required_build_manifest_schema"), str) or
            not provenance["required_build_manifest_schema"]):
        raise ValueError("required build manifest schema must be explicit")
    if provenance.get("require_exact_effective_params") is not True:
        raise ValueError("exact effective FAST-LIVO parameters must be required")
    for key in ("effective_param_ephemeral_allowlist",
                "private_camera_param_ephemeral_allowlist"):
        if not _valid_allowlist(provenance.get(key)):
            raise ValueError(
                "%s must be unique explicit JSON-pointer-like paths without wildcards" % key)

    evidence = profile.get("evidence")
    _exact_keys(evidence, EVIDENCE_KEYS, "evidence")
    if not isinstance(evidence.get("require_source_identity"), bool):
        raise ValueError("require_source_identity must be an explicit boolean")
    if not _finite_positive(evidence.get("max_duration_overrun_s")):
        raise ValueError("max_duration_overrun_s must be finite and positive")
    sources = evidence.get("source_paths")
    _exact_keys(sources, EVIDENCE_SOURCE_KEYS, "evidence.source_paths")
    if not all(isinstance(value, str) for value in sources.values()):
        raise ValueError("evidence source paths must be strings")
    manifest_paths = evidence.get("source_manifest_paths")
    _exact_keys(manifest_paths, EVIDENCE_MANIFEST_PATH_KEYS,
                "evidence.source_manifest_paths")
    if not all(isinstance(value, str) and value for value in manifest_paths.values()):
        raise ValueError("evidence manifest paths must be nonempty strings")
    return True


def _percentile(values, percentile):
    values = sorted(float(value) for value in values if _strict_number(value))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    rank = (len(values) - 1) * float(percentile) / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return values[lower]
    fraction = rank - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def _distribution(values):
    finite = [float(value) for value in values if _strict_number(value)]
    if not finite:
        return {"count": 0, "min": None, "mean": None, "p50": None,
                "p95": None, "p99": None, "max": None}
    return {
        "count": len(finite), "min": min(finite), "mean": statistics.mean(finite),
        "p50": _percentile(finite, 50), "p95": _percentile(finite, 95),
        "p99": _percentile(finite, 99), "max": max(finite)}


def _jacobi_eigenvalues(matrix, tolerance=1.0e-14, iterations=256):
    """Eigenvalues of a small real symmetric matrix, without numpy."""
    values = [list(map(float, row)) for row in matrix]
    size = len(values)
    for _ in range(iterations):
        largest = 0.0
        pivot = (0, 0)
        for row in range(size):
            for col in range(row + 1, size):
                magnitude = abs(values[row][col])
                if magnitude > largest:
                    largest = magnitude
                    pivot = (row, col)
        if largest <= tolerance:
            break
        row, col = pivot
        angle = 0.5 * math.atan2(
            2.0 * values[row][col], values[col][col] - values[row][row])
        cosine = math.cos(angle)
        sine = math.sin(angle)
        arr = values[row][row]
        acc = values[col][col]
        arc = values[row][col]
        values[row][row] = cosine * cosine * arr - 2.0 * sine * cosine * arc + sine * sine * acc
        values[col][col] = sine * sine * arr + 2.0 * sine * cosine * arc + cosine * cosine * acc
        values[row][col] = values[col][row] = 0.0
        for other in range(size):
            if other in (row, col):
                continue
            old_row = values[other][row]
            old_col = values[other][col]
            values[other][row] = values[row][other] = cosine * old_row - sine * old_col
            values[other][col] = values[col][other] = sine * old_row + cosine * old_col
    return [values[index][index] for index in range(size)]


def _covariance_quality(values, config):
    base = {
        "available": values is not None, "finite": False, "nonzero": False,
        "positive_diagonal": False, "symmetric": False, "psd": False,
        "eigenvalue_floor": False, "within_bounds": False,
        "min_eigenvalue": None, "max_asymmetry": None,
        "minimum_diagonal": None, "maximum_diagonal": None,
        "maximum_absolute_value": None}
    if values is None:
        return base
    values = list(values)
    if len(values) != 36 or not all(_strict_number(value) for value in values):
        return base
    numeric = [float(value) for value in values]
    base["finite"] = True
    base["nonzero"] = any(abs(value) > 0.0 for value in numeric)
    diagonal = [numeric[index * 6 + index] for index in range(6)]
    base["positive_diagonal"] = all(value > 0.0 for value in diagonal)
    base["minimum_diagonal"] = min(diagonal)
    base["maximum_diagonal"] = max(diagonal)
    base["maximum_absolute_value"] = max(abs(value) for value in numeric)
    asymmetry = max(
        abs(numeric[row * 6 + col] - numeric[col * 6 + row])
        for row in range(6) for col in range(6))
    base["max_asymmetry"] = asymmetry
    base["symmetric"] = asymmetry <= float(config["covariance_symmetry_tolerance"])
    base["within_bounds"] = (
        all(float(config["covariance_min_diagonal"]) <= value <=
            float(config["covariance_max_diagonal"]) for value in diagonal) and
        base["maximum_absolute_value"] <= float(config["covariance_max_abs"]))
    if base["symmetric"]:
        matrix = [[numeric[row * 6 + col] for col in range(6)] for row in range(6)]
        eigenvalues = _jacobi_eigenvalues(matrix)
        base["min_eigenvalue"] = min(eigenvalues)
        base["psd"] = base["min_eigenvalue"] >= -float(config["covariance_psd_tolerance"])
        base["eigenvalue_floor"] = (
            base["min_eigenvalue"] >= float(config["covariance_min_eigenvalue"]))
    return base


def _imu_covariance_quality(values, config):
    """Validate a ROS Imu 3x3 covariance; ``-1`` means explicitly unknown."""
    base = {
        "available": values is not None, "known": False, "finite": False,
        "nonzero": False, "positive_diagonal": False, "symmetric": False,
        "psd": False, "eigenvalue_floor": False, "within_bounds": False,
        "valid": False, "min_eigenvalue": None, "max_asymmetry": None,
        "minimum_diagonal": None, "maximum_diagonal": None}
    if values is None:
        return base
    values = list(values)
    if len(values) != 9 or not all(_strict_number(value) for value in values):
        return base
    numeric = [float(value) for value in values]
    base["finite"] = True
    base["nonzero"] = any(abs(value) > 0.0 for value in numeric)
    base["known"] = numeric[0] != -1.0 and base["nonzero"]
    if not base["known"]:
        return base
    diagonal = [numeric[index * 3 + index] for index in range(3)]
    base["positive_diagonal"] = all(value > 0.0 for value in diagonal)
    base["minimum_diagonal"] = min(diagonal)
    base["maximum_diagonal"] = max(diagonal)
    asymmetry = max(
        abs(numeric[row * 3 + col] - numeric[col * 3 + row])
        for row in range(3) for col in range(3))
    base["max_asymmetry"] = asymmetry
    base["symmetric"] = asymmetry <= float(config["covariance_symmetry_tolerance"])
    base["within_bounds"] = (
        all(float(config["covariance_min_diagonal"]) <= value <=
            float(config["covariance_max_diagonal"]) for value in diagonal) and
        max(abs(value) for value in numeric) <= float(config["covariance_max_abs"]))
    if base["symmetric"]:
        matrix = [[numeric[row * 3 + col] for col in range(3)] for row in range(3)]
        eigenvalues = _jacobi_eigenvalues(matrix)
        base["min_eigenvalue"] = min(eigenvalues)
        base["psd"] = base["min_eigenvalue"] >= -float(
            config["covariance_psd_tolerance"])
        base["eigenvalue_floor"] = (
            base["min_eigenvalue"] >= float(config["covariance_min_eigenvalue"]))
    base["valid"] = all(
        base[key] for key in (
            "available", "known", "finite", "nonzero", "positive_diagonal",
            "symmetric", "psd", "eigenvalue_floor", "within_bounds"))
    return base


def _vector_quality(values, length):
    available = values is not None
    values = list(values) if available else []
    finite = len(values) == length and all(_strict_number(value) for value in values)
    return {"available": available, "finite": finite}


def _pose_quality(position, orientation, tolerance):
    position_quality = _vector_quality(position, 3)
    orientation_quality = _vector_quality(orientation, 4)
    norm = (math.sqrt(sum(float(value) ** 2 for value in orientation))
            if orientation_quality["finite"] else None)
    qnorm = norm is not None and abs(norm - 1.0) <= float(tolerance)
    return {
        "available": position_quality["available"] and orientation_quality["available"],
        "finite": position_quality["finite"] and orientation_quality["finite"],
        "quaternion_norm": norm, "quaternion_normalized": qnorm,
        "valid": position_quality["finite"] and orientation_quality["finite"] and qnorm}


def _twist_quality(linear, angular):
    linear_quality = _vector_quality(linear, 3)
    angular_quality = _vector_quality(angular, 3)
    return {
        "available": linear_quality["available"] and angular_quality["available"],
        "finite": linear_quality["finite"] and angular_quality["finite"],
        "valid": linear_quality["finite"] and angular_quality["finite"]}


def _json_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return value
    if math.isnan(float(value)):
        return "NaN"
    if math.isinf(float(value)):
        return "+Inf" if float(value) > 0 else "-Inf"
    return value


def _json_vector(values):
    return None if values is None else [_json_number(value) for value in values]


class StreamAccumulator(object):
    """Accumulate exact timing, payload, covariance, and recomputable raw rows."""

    def __init__(self, name, config, window_start_wall_ns=None):
        self.name = str(name)
        self.config = dict(config)
        self.window_start_wall_ns = (None if window_start_wall_ns is None
                                     else int(window_start_wall_ns))
        self.sample_count = 0
        self.first_arrival_ns = None
        self.last_arrival_ns = None
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.arrival_intervals_s = []
        self.stamp_intervals_s = []
        self.latencies_s = []
        self.zero_stamp_count = 0
        self.duplicate_stamp_count = 0
        self.backward_stamp_count = 0
        self.arrival_backward_count = 0
        self.arrival_gap_count = 0
        self.max_arrival_gap_s = 0.0
        self.gap_count = 0
        self.max_gap_s = 0.0
        self.frame_ids = set()
        self.child_frame_ids = set()
        self.frame_mismatch_count = 0
        self.child_frame_mismatch_count = 0
        quality_keys = (
            "available", "finite", "nonzero", "positive_diagonal", "symmetric",
            "psd", "eigenvalue_floor", "within_bounds", "valid")
        self.pose_cov_quality_counts = {key: 0 for key in quality_keys}
        self.twist_cov_quality_counts = {key: 0 for key in quality_keys}
        self.pose_payload_counts = {key: 0 for key in ("available", "finite", "qnorm", "valid")}
        self.twist_payload_counts = {key: 0 for key in ("available", "finite", "valid")}
        self.imu_payload_counts = {key: 0 for key in ("available", "finite", "valid")}
        self.imu_orientation_counts = {
            key: 0 for key in ("available", "finite", "qnorm", "valid")}
        imu_cov_keys = {
            "available", "known", "finite", "nonzero", "positive_diagonal",
            "symmetric", "psd", "eigenvalue_floor", "within_bounds", "valid"}
        self.imu_cov_quality_counts = {
            name: {key: 0 for key in imu_cov_keys}
            for name in ("orientation", "angular_velocity", "linear_acceleration")}
        self.imu_cov_min_eigenvalues = {
            name: [] for name in self.imu_cov_quality_counts}
        self.quaternion_norms = []
        self.pose_min_eigenvalues = []
        self.twist_min_eigenvalues = []
        self.raw_records = []

    def observe(self, stamp_ns, arrival_ros_ns, frame_id, child_frame_id="",
                pose_covariance=None, twist_covariance=None, arrival_wall_ns=None,
                pose_position=None, pose_orientation=None, twist_linear=None,
                twist_angular=None, imu_angular_velocity=None,
                imu_linear_acceleration=None, imu_orientation=None,
                imu_orientation_covariance=None,
                imu_angular_velocity_covariance=None,
                imu_linear_acceleration_covariance=None):
        stamp_ns = int(stamp_ns)
        arrival_ros_ns = int(arrival_ros_ns)
        arrival_wall_ns = int(arrival_ros_ns if arrival_wall_ns is None else arrival_wall_ns)
        frame_id = str(frame_id or "")
        child_frame_id = str(child_frame_id or "")
        if self.window_start_wall_ns is None:
            self.window_start_wall_ns = arrival_wall_ns
        if self.last_arrival_ns is not None:
            arrival_dt = _seconds(arrival_wall_ns - self.last_arrival_ns)
            if arrival_dt < 0.0:
                self.arrival_backward_count += 1
            else:
                self.arrival_intervals_s.append(arrival_dt)
                if arrival_dt > float(self.config["max_gap_s"]):
                    self.arrival_gap_count += 1
                self.max_arrival_gap_s = max(self.max_arrival_gap_s, arrival_dt)
        if self.last_stamp_ns is not None:
            stamp_dt = _seconds(stamp_ns - self.last_stamp_ns)
            if stamp_dt < 0.0:
                self.backward_stamp_count += 1
            elif stamp_dt == 0.0:
                self.duplicate_stamp_count += 1
            else:
                self.stamp_intervals_s.append(stamp_dt)
                if stamp_dt > float(self.config["max_gap_s"]):
                    self.gap_count += 1
                self.max_gap_s = max(self.max_gap_s, stamp_dt)
        if stamp_ns <= 0:
            self.zero_stamp_count += 1
        else:
            self.latencies_s.append(_seconds(arrival_ros_ns - stamp_ns))
        self.sample_count += 1
        if self.first_arrival_ns is None:
            self.first_arrival_ns = arrival_wall_ns
            self.first_stamp_ns = stamp_ns
        self.last_arrival_ns = arrival_wall_ns
        self.last_stamp_ns = stamp_ns
        self.frame_ids.add(frame_id)
        self.child_frame_ids.add(child_frame_id)
        if frame_id != str(self.config["expected_frame_id"]):
            self.frame_mismatch_count += 1
        if child_frame_id != str(self.config["expected_child_frame_id"]):
            self.child_frame_mismatch_count += 1

        pose_cov = _covariance_quality(pose_covariance, self.config)
        twist_cov = _covariance_quality(twist_covariance, self.config)
        cov_required = ("available", "finite", "nonzero", "positive_diagonal",
                        "symmetric", "psd", "eigenvalue_floor", "within_bounds")
        for quality, counters, eigenvalues in (
                (pose_cov, self.pose_cov_quality_counts, self.pose_min_eigenvalues),
                (twist_cov, self.twist_cov_quality_counts, self.twist_min_eigenvalues)):
            for key in cov_required:
                if quality[key]:
                    counters[key] += 1
            valid = all(quality[key] for key in cov_required)
            if valid:
                counters["valid"] += 1
            if quality["min_eigenvalue"] is not None:
                eigenvalues.append(quality["min_eigenvalue"])
        pose = _pose_quality(
            pose_position, pose_orientation, self.config["quaternion_norm_tolerance"])
        twist = _twist_quality(twist_linear, twist_angular)
        imu = _twist_quality(imu_angular_velocity, imu_linear_acceleration)
        imu_orientation_quality = _pose_quality(
            [0.0, 0.0, 0.0], imu_orientation,
            self.config["quaternion_norm_tolerance"])
        for key, target in (("available", "available"), ("finite", "finite"),
                            ("quaternion_normalized", "qnorm"), ("valid", "valid")):
            if pose[key]:
                self.pose_payload_counts[target] += 1
        for key in ("available", "finite", "valid"):
            if twist[key]:
                self.twist_payload_counts[key] += 1
            if imu[key]:
                self.imu_payload_counts[key] += 1
        for key, target in (("available", "available"), ("finite", "finite"),
                            ("quaternion_normalized", "qnorm"), ("valid", "valid")):
            if imu_orientation_quality[key]:
                self.imu_orientation_counts[target] += 1
        for name, values in (
                ("orientation", imu_orientation_covariance),
                ("angular_velocity", imu_angular_velocity_covariance),
                ("linear_acceleration", imu_linear_acceleration_covariance)):
            quality = _imu_covariance_quality(values, self.config)
            for key in self.imu_cov_quality_counts[name]:
                if quality[key]:
                    self.imu_cov_quality_counts[name][key] += 1
            if quality["min_eigenvalue"] is not None:
                self.imu_cov_min_eigenvalues[name].append(quality["min_eigenvalue"])
        if pose["quaternion_norm"] is not None:
            self.quaternion_norms.append(pose["quaternion_norm"])
        self.raw_records.append({
            "kind": "stream", "stream": self.name, "stamp_ns": stamp_ns,
            "arrival_ros_ns": arrival_ros_ns, "arrival_wall_ns": arrival_wall_ns,
            "frame_id": frame_id, "child_frame_id": child_frame_id,
            "pose_position": _json_vector(pose_position),
            "pose_orientation_xyzw": _json_vector(pose_orientation),
            "twist_linear": _json_vector(twist_linear),
            "twist_angular": _json_vector(twist_angular),
            "imu_angular_velocity": _json_vector(imu_angular_velocity),
            "imu_linear_acceleration": _json_vector(imu_linear_acceleration),
            "imu_orientation_xyzw": _json_vector(imu_orientation),
            "imu_orientation_covariance": _json_vector(imu_orientation_covariance),
            "imu_angular_velocity_covariance": _json_vector(
                imu_angular_velocity_covariance),
            "imu_linear_acceleration_covariance": _json_vector(
                imu_linear_acceleration_covariance),
            "pose_covariance": _json_vector(pose_covariance),
            "twist_covariance": _json_vector(twist_covariance)})

    def summarize(self, now_wall_ns, now_ros_ns=None, requested_duration_s=None):
        now_wall_ns = int(now_wall_ns)
        now_ros_ns = int(now_wall_ns if now_ros_ns is None else now_ros_ns)
        arrival_duration = (_seconds(self.last_arrival_ns - self.first_arrival_ns)
                            if self.sample_count > 1 and self.last_arrival_ns > self.first_arrival_ns
                            else None)
        stamp_duration = (_seconds(self.last_stamp_ns - self.first_stamp_ns)
                          if self.sample_count > 1 and self.last_stamp_ns > self.first_stamp_ns
                          else None)
        arrival_rate = ((self.sample_count - 1) / arrival_duration if arrival_duration else None)
        stamp_rate = ((self.sample_count - 1) / stamp_duration if stamp_duration else None)
        idle_s = (_seconds(now_wall_ns - self.last_arrival_ns)
                  if self.last_arrival_ns is not None else None)
        header_age_s = (_seconds(now_ros_ns - self.last_stamp_ns)
                        if self.last_stamp_ns is not None and self.last_stamp_ns > 0 else None)
        first_delay_s = (_seconds(self.first_arrival_ns - self.window_start_wall_ns)
                         if self.first_arrival_ns is not None and
                         self.window_start_wall_ns is not None else None)
        duration = (float(requested_duration_s) if requested_duration_s is not None
                    else arrival_duration)
        coverage = (min(1.0, max(0.0, arrival_duration / duration))
                    if arrival_duration is not None and duration and duration > 0 else None)
        max_idle = float(self.config["max_idle_s"])
        max_latency = float(self.config["max_abs_header_latency_s"])
        gates = {
            "minimum_samples": self.sample_count >= int(self.config["min_samples"]),
            "minimum_arrival_rate": arrival_rate is not None and arrival_rate >= float(self.config["min_rate_hz"]),
            "minimum_header_stamp_rate": stamp_rate is not None and stamp_rate >= float(self.config["min_rate_hz"]),
            "maximum_header_gap": self.gap_count == 0,
            "maximum_arrival_gap": self.arrival_gap_count == 0,
            "arrival_fresh_at_finish": idle_s is not None and 0.0 <= idle_s <= max_idle,
            "header_fresh_at_finish": header_age_s is not None and abs(header_age_s) <= max_idle + max_latency,
            "nonzero_monotonic_unique_stamps": self.zero_stamp_count == 0 and self.backward_stamp_count == 0 and self.duplicate_stamp_count == 0,
            "arrival_clock_monotonic": self.arrival_backward_count == 0,
            "frame_id_exact": self.frame_mismatch_count == 0,
            "child_frame_id_exact": self.child_frame_mismatch_count == 0,
            "header_latency": bool(self.latencies_s) and max(abs(value) for value in self.latencies_s) <= max_latency,
            "first_sample_delay": first_delay_s is not None and 0.0 <= first_delay_s <= float(self.config["max_first_sample_delay_s"]),
            "window_coverage": coverage is not None and coverage >= float(self.config["min_window_coverage_ratio"]),
            "pose_payload": (not self.config["require_pose_payload"] or
                             (self.sample_count > 0 and self.pose_payload_counts["valid"] == self.sample_count)),
            "twist_payload": (not self.config["require_twist_payload"] or
                              (self.sample_count > 0 and self.twist_payload_counts["valid"] == self.sample_count)),
            "imu_payload": (not self.config["require_imu_payload"] or
                            (self.sample_count > 0 and self.imu_payload_counts["valid"] == self.sample_count)),
            "imu_orientation": (not self.config["require_imu_orientation"] or
                                (self.sample_count > 0 and
                                 self.imu_orientation_counts["valid"] == self.sample_count)),
            "imu_covariance": (not self.config["require_imu_covariance"] or
                               (self.sample_count > 0 and all(
                                   counts["valid"] == self.sample_count
                                   for counts in self.imu_cov_quality_counts.values()))),
            "pose_covariance": (not self.config["require_pose_covariance"] or
                                (self.sample_count > 0 and self.pose_cov_quality_counts["valid"] == self.sample_count)),
            "twist_covariance": (not self.config["require_twist_covariance"] or
                                 (self.sample_count > 0 and self.twist_cov_quality_counts["valid"] == self.sample_count)),
        }
        return {
            "name": self.name, "topic": self.config["topic"],
            "message_type": self.config["message_type"],
            "contract": dict(self.config), "sample_count": self.sample_count,
            "arrival_rate_hz": arrival_rate, "header_stamp_rate_hz": stamp_rate,
            "arrival_clock_basis": "monotonic_wall",
            "header_latency_clock_basis": "ROS arrival time minus ROS header stamp",
            "first_sample_delay_s": first_delay_s, "window_coverage_ratio": coverage,
            "arrival_interval_s": _distribution(self.arrival_intervals_s),
            "header_interval_s": _distribution(self.stamp_intervals_s),
            "arrival_minus_header_s": _distribution(self.latencies_s),
            "idle_at_finish_s": idle_s, "header_age_at_finish_s": header_age_s,
            "zero_stamp_count": self.zero_stamp_count,
            "duplicate_stamp_count": self.duplicate_stamp_count,
            "backward_stamp_count": self.backward_stamp_count,
            "arrival_backward_count": self.arrival_backward_count,
            "arrival_gap_count": self.arrival_gap_count,
            "maximum_observed_arrival_gap_s": self.max_arrival_gap_s if self.arrival_intervals_s else None,
            "gap_count": self.gap_count,
            "maximum_observed_gap_s": self.max_gap_s if self.stamp_intervals_s else None,
            "frame_ids": sorted(self.frame_ids), "child_frame_ids": sorted(self.child_frame_ids),
            "frame_mismatch_count": self.frame_mismatch_count,
            "child_frame_mismatch_count": self.child_frame_mismatch_count,
            "payload": {
                "pose": dict(self.pose_payload_counts),
                "twist": dict(self.twist_payload_counts),
                "imu": dict(self.imu_payload_counts),
                "imu_orientation": dict(self.imu_orientation_counts),
                "quaternion_norm": _distribution(self.quaternion_norms)},
            "covariance": {
                "pose": dict(self.pose_cov_quality_counts),
                "twist": dict(self.twist_cov_quality_counts),
                "imu": {name: dict(counts) for name, counts in
                        sorted(self.imu_cov_quality_counts.items())},
                "imu_min_eigenvalue": {
                    name: _distribution(values) for name, values in
                    sorted(self.imu_cov_min_eigenvalues.items())},
                "pose_min_eigenvalue": _distribution(self.pose_min_eigenvalues),
                "twist_min_eigenvalue": _distribution(self.twist_min_eigenvalues)},
            "gates": gates, "status": "PASS" if all(gates.values()) else "FAIL"}


def _strict_uint_text(value):
    text = str(value)
    if not text or not text.isdigit():
        return None
    return int(text)


def _strict_int_text(value):
    text = str(value)
    digits = text[1:] if text.startswith("-") else text
    if not digits or not digits.isdigit():
        return None
    return int(text)


def _valid_sha256(value):
    return (isinstance(value, str) and len(value) == 64 and
            all(character in "0123456789abcdef" for character in value))


def canonical_value_sha256(value):
    try:
        payload = json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
            allow_nan=False).encode("utf-8")
    except (TypeError, ValueError):
        return None
    return hashlib.sha256(payload).hexdigest()


class IdentityAccumulator(object):
    """Require one typed session/reset/correction identity record per epoch."""

    def __init__(self, config, expected_producer_identity=None):
        self.config = dict(config)
        self.expected_producer_identity = dict(expected_producer_identity or {})
        self.records = []
        self.raw_records = []

    @property
    def join_records(self):
        return list(self.records)

    def observe_join(self, name, level, message, values, stamp_ns,
                     arrival_wall_ns, arrival_ros_ns):
        if str(name) != self.config["status_name"]:
            return
        values_raw, normalized, duplicate_keys, values_well_formed = \
            _normalize_key_values(values)
        parsed = None
        if values_well_formed and not duplicate_keys and set(normalized) == IDENTITY_VALUE_KEYS:
            reset = _strict_uint_text(normalized.get("reset_counter"))
            sequence = _strict_uint_text(normalized.get("correction_sequence"))
            correction_stamp = _strict_uint_text(normalized.get("correction_stamp_ns"))
            producer_identity = {
                key: normalized.get(key) for key in PRODUCER_IDENTITY_VALUE_KEYS}
            producer_gate = (
                set(self.expected_producer_identity) == PRODUCER_IDENTITY_VALUE_KEYS and
                all(_valid_sha256(value) and
                    value == self.expected_producer_identity.get(key)
                    for key, value in producer_identity.items()))
            if (normalized.get("schema") == self.config["status_schema"] and
                    bool(normalized.get("session_id", "").strip()) and
                    reset is not None and sequence is not None and
                    correction_stamp is not None and correction_stamp > 0 and
                    producer_gate):
                parsed = {
                    "session_id": normalized["session_id"],
                    "reset_counter": reset, "correction_sequence": sequence,
                    "correction_stamp_ns": correction_stamp,
                    "producer_identity": producer_identity}
        record = {
            "name": str(name), "level": level, "message": str(message),
            "values": normalized, "values_raw": values_raw,
            "duplicate_value_keys": duplicate_keys,
            "values_well_formed": values_well_formed,
            "parsed": parsed, "stamp_ns": int(stamp_ns),
            "arrival_wall_ns": int(arrival_wall_ns),
            "arrival_ros_ns": int(arrival_ros_ns)}
        self.records.append(record)
        raw = dict(record)
        raw.pop("parsed", None)
        raw["kind"] = "typed_identity_join"
        self.raw_records.append(raw)

    def summarize(self, now_wall_ns):
        valid = [item for item in self.records if item["parsed"] is not None]
        parsed = [item["parsed"] for item in valid]
        stamps = [item["stamp_ns"] for item in self.records]
        arrivals = [item["arrival_wall_ns"] for item in self.records]
        latencies = [_seconds(item["arrival_ros_ns"] - item["stamp_ns"])
                     for item in self.records]
        idle = (_seconds(int(now_wall_ns) - arrivals[-1]) if arrivals else None)
        sessions = [item["session_id"] for item in parsed]
        resets = [item["reset_counter"] for item in parsed]
        sequences = [item["correction_sequence"] for item in parsed]
        correction_stamps = [item["correction_stamp_ns"] for item in parsed]
        gates = {
            "typed_exact_schema": (
                len(self.records) >= int(self.config["min_samples"]) and
                len(valid) == len(self.records)),
            "no_duplicate_key_values": bool(self.records) and all(
                item["values_well_formed"] and not item["duplicate_value_keys"]
                for item in self.records),
            "all_status_levels_ok": bool(self.records) and all(
                isinstance(item["level"], int) and not isinstance(item["level"], bool) and
                item["level"] == 0 for item in self.records),
            "header_stamps_monotonic": bool(stamps) and all(item > 0 for item in stamps) and
                all(right > left for left, right in zip(stamps, stamps[1:])),
            "arrival_clock_monotonic": bool(arrivals) and all(
                right > left for left, right in zip(arrivals, arrivals[1:])),
            "header_latency": bool(latencies) and all(
                abs(value) <= float(self.config["max_abs_header_latency_s"])
                for value in latencies),
            "fresh_at_finish": idle is not None and
                0 <= idle <= float(self.config["max_idle_s"]),
            "single_nonempty_session": bool(sessions) and len(set(sessions)) == 1,
            "no_reset_during_window": bool(resets) and len(set(resets)) == 1,
            "correction_sequence_contiguous": bool(sequences) and all(
                right == left + 1 for left, right in zip(sequences, sequences[1:])),
            "correction_stamps_monotonic_unique": bool(correction_stamps) and all(
                right > left for left, right in
                zip(correction_stamps, correction_stamps[1:])),
        }
        return {
            "contract": dict(self.config), "sample_count": len(self.records),
            "valid_typed_record_count": len(valid),
            "session_values": list(dict.fromkeys(sessions)),
            "reset_values": list(dict.fromkeys(resets)),
            "correction_sequences": sequences,
            "correction_stamps_ns": correction_stamps,
            "expected_producer_identity": dict(self.expected_producer_identity),
            "idle_at_finish_s": idle,
            "arrival_minus_header_s": _distribution(latencies),
            "gates": gates, "status": "PASS" if all(gates.values()) else "FAIL"}


class LinkageAccumulator(object):
    """Prove correction→typed identity→propagation linkage one-to-one."""

    def __init__(self, config):
        self.config = dict(config)
        self.streams = {}
        self.raw_records = []

    def observe_stream(self, name, stamp_ns, arrival_wall_ns):
        record = (int(stamp_ns), int(arrival_wall_ns))
        self.streams.setdefault(str(name), []).append(record)

    def summarize(self, identity_records):
        corrections = self.streams.get(self.config["correction_stream"], [])
        propagated = self.streams.get(self.config["propagated_stream"], [])
        parsed_identity = [item for item in identity_records
                           if isinstance(item.get("parsed"), dict)]
        header_deltas = []
        for stamp, _arrival in corrections:
            header_deltas.append(min(
                (abs(_seconds(stamp - other_stamp)) for other_stamp, _ in propagated),
                default=None))
        propagation_gate = bool(corrections) and bool(propagated) and all(
            delta is not None and delta <= float(self.config["max_header_delta_s"])
            for delta in header_deltas)

        stamp_limit_ns = int(
            float(self.config["max_identity_correction_stamp_delta_s"]) * 1.0e9)
        used = set()
        matched = []
        ambiguous = 0
        for correction_stamp, correction_arrival in corrections:
            candidates = [
                index for index, record in enumerate(parsed_identity)
                if index not in used and abs(
                    correction_stamp - record["parsed"]["correction_stamp_ns"])
                <= stamp_limit_ns]
            if len(candidates) != 1:
                ambiguous += 1
                continue
            index = candidates[0]
            used.add(index)
            matched.append((correction_stamp, correction_arrival, parsed_identity[index]))
        stamp_deltas = [
            abs(_seconds(stamp - record["parsed"]["correction_stamp_ns"]))
            for stamp, _arrival, record in matched]
        arrival_deltas = [
            abs(_seconds(arrival - record["arrival_wall_ns"]))
            for _stamp, arrival, record in matched]
        one_to_one = (
            bool(corrections) and len(identity_records) == len(corrections) and
            len(parsed_identity) == len(corrections) and len(matched) == len(corrections) and
            len(used) == len(parsed_identity) and ambiguous == 0)
        sequences = [record["parsed"]["correction_sequence"]
                     for _stamp, _arrival, record in matched]
        gates = {
            "correction_to_propagated_header_link": propagation_gate,
            "typed_identity_join_required": self.config["require_typed_identity_join"] is True,
            "one_typed_identity_per_correction": one_to_one,
            "identity_correction_stamp_link": one_to_one and all(
                value <= float(self.config["max_identity_correction_stamp_delta_s"])
                for value in stamp_deltas),
            "identity_correction_arrival_link": one_to_one and all(
                value <= float(self.config["max_status_arrival_delta_s"])
                for value in arrival_deltas),
            "identity_sequence_contiguous": one_to_one and all(
                right == left + 1 for left, right in zip(sequences, sequences[1:])),
        }
        return {
            "contract": dict(self.config), "correction_count": len(corrections),
            "propagated_count": len(propagated),
            "typed_identity_count": len(identity_records),
            "matched_identity_count": len(matched), "ambiguous_match_count": ambiguous,
            "nearest_propagated_header_delta_s": _distribution(header_deltas),
            "identity_correction_stamp_delta_s": _distribution(stamp_deltas),
            "identity_correction_arrival_delta_s": _distribution(arrival_deltas),
            "correction_sequences": sequences, "gates": gates,
            "status": "PASS" if all(gates.values()) else "FAIL"}


class HybridImuAccumulator(object):
    """Latch readiness/faults and typed mapping/clock provenance."""

    _FAULT_PREFIXES = ("drop_", "invalid_", "fault_")
    _FAULT_SUFFIXES = ("_rollback", "_error", "_overflow", "_timeout")

    def __init__(self, config):
        self.config = dict(config)
        self.ready_samples = []
        self.diagnostics = []
        self.mapping_starts = []
        self.timesync_domains = []
        self.raw_records = []

    def observe_ready(self, value, arrival_wall_ns):
        record = (bool(value), int(arrival_wall_ns))
        self.ready_samples.append(record)
        self.raw_records.append({"kind": "hybrid_ready", "value": bool(value),
                                 "arrival_wall_ns": int(arrival_wall_ns)})

    def observe_diagnostic(self, name, level, message, values, stamp_ns,
                           arrival_wall_ns, arrival_ros_ns=None):
        if str(name) != self.config["diagnostic_name"]:
            return
        values_raw, normalized, duplicate_keys, values_well_formed = \
            _normalize_key_values(values)
        record = {
            "name": str(name), "level": level, "message": str(message),
            "values": normalized, "values_raw": values_raw,
            "duplicate_value_keys": duplicate_keys,
            "values_well_formed": values_well_formed,
            "stamp_ns": int(stamp_ns), "arrival_wall_ns": int(arrival_wall_ns),
            "arrival_ros_ns": int(stamp_ns if arrival_ros_ns is None else arrival_ros_ns)}
        self.diagnostics.append(record)
        raw = dict(record)
        raw["kind"] = "hybrid_diagnostic"
        self.raw_records.append(raw)

    def observe_mapping_start(self, value, arrival_wall_ns):
        if self.config["mapping_start_message_type"] == "std_msgs/UInt32":
            valid = isinstance(value, int) and not isinstance(value, bool) and value >= 0
            normalized = int(value) if valid else None
        else:
            normalized = str(value).strip()
            valid = bool(normalized)
        record = {"value": normalized, "valid": valid,
                  "arrival_wall_ns": int(arrival_wall_ns)}
        self.mapping_starts.append(record)
        self.raw_records.append(dict(record, kind="mapping_start"))

    def observe_timesync_domain(self, name, level, message, values, stamp_ns,
                                arrival_wall_ns, arrival_ros_ns,
                                publisher_runtime_identity=None):
        if str(name) != self.config["timesync_domain_status_name"]:
            return
        values_raw, normalized, duplicate_keys, values_well_formed = \
            _normalize_key_values(values)
        record = {
            "name": str(name), "level": level, "message": str(message),
            "values": normalized, "values_raw": values_raw,
            "duplicate_value_keys": duplicate_keys,
            "values_well_formed": values_well_formed,
            "stamp_ns": int(stamp_ns), "arrival_wall_ns": int(arrival_wall_ns),
            "arrival_ros_ns": int(arrival_ros_ns),
            "publisher_runtime_identity": dict(
                publisher_runtime_identity or {})}
        self.timesync_domains.append(record)
        self.raw_records.append(dict(record, kind="hybrid_timesync_domain"))

    @staticmethod
    def _as_bool(value):
        return str(value).strip().lower() == "true"

    @staticmethod
    def _counter(value):
        return _strict_uint_text(value)

    @classmethod
    def _is_fault_key(cls, key):
        return key.startswith(cls._FAULT_PREFIXES) or key.endswith(cls._FAULT_SUFFIXES)

    def summarize(self, now_wall_ns, output_stream_report, expected_source_hashes=None,
                  expected_session_ids=None, expected_reset_counters=None):
        now_wall_ns = int(now_wall_ns)
        expected_source_hashes = dict(expected_source_hashes or {})
        expected_session_ids = list(expected_session_ids or [])
        expected_reset_counters = list(expected_reset_counters or [])
        true_arrivals = [arrival for value, arrival in self.ready_samples if value]
        first_ready = true_arrivals[0] if true_arrivals else None
        false_after_ready = sum(
            not value and first_ready is not None and arrival >= first_ready
            for value, arrival in self.ready_samples)
        final_ready = bool(self.ready_samples) and self.ready_samples[-1][0] is True
        diagnostics = [item for item in self.diagnostics
                       if first_ready is not None and item["arrival_wall_ns"] >= first_ready]
        intervals = [_seconds(right["arrival_wall_ns"] - left["arrival_wall_ns"])
                     for left, right in zip(diagnostics, diagnostics[1:])]
        diagnostic_fresh = bool(diagnostics) and 0 <= _seconds(
            now_wall_ns - diagnostics[-1]["arrival_wall_ns"]) <= float(
                self.config["diagnostic_max_idle_s"])
        diagnostic_continuous = len(diagnostics) >= 2 and all(
            0 < value <= float(self.config["diagnostic_max_gap_s"])
            for value in intervals)
        diagnostic_ok = bool(diagnostics) and all(
            isinstance(item["level"], int) and not isinstance(item["level"], bool) and
            item["level"] == 0 and item["values_well_formed"] and
            not item["duplicate_value_keys"] and
            self._as_bool(item["values"].get("ready", "false"))
            for item in diagnostics)
        diagnostic_keyvalues_unique = bool(diagnostics) and all(
            item["values_well_formed"] and not item["duplicate_value_keys"]
            for item in diagnostics)
        diagnostic_header_gate = bool(diagnostics) and all(
            item["stamp_ns"] > 0 for item in diagnostics) and all(
                right["stamp_ns"] > left["stamp_ns"]
                for left, right in zip(diagnostics, diagnostics[1:]))
        diagnostic_header_ages = [
            _seconds(item["arrival_ros_ns"] - item["stamp_ns"])
            for item in diagnostics]
        diagnostic_header_latency_gate = bool(diagnostic_header_ages) and all(
            abs(value) <= float(self.config["diagnostic_max_idle_s"])
            for value in diagnostic_header_ages)

        required_fault_keys = set(self.config["required_fault_counter_keys"])
        per_record_fault_keys = [
            {key for key in item["values"] if self._is_fault_key(key)}
            for item in diagnostics]
        counter_schema = diagnostic_keyvalues_unique and all(
            keys == required_fault_keys for keys in per_record_fault_keys)
        counter_values = {}
        counters_valid = counter_schema
        counters_monotonic = counter_schema
        for key in sorted(required_fault_keys):
            values = [self._counter(item["values"].get(key)) for item in diagnostics]
            counter_values[key] = values
            if not values or any(value is None for value in values):
                counters_valid = False
                counters_monotonic = False
            elif not all(right >= left for left, right in zip(values, values[1:])):
                counters_monotonic = False
        fault_baseline = {key: (values[0] if values and None not in values else None)
                          for key, values in counter_values.items()}
        fault_final = {key: (values[-1] if values and None not in values else None)
                       for key, values in counter_values.items()}
        fault_deltas = {
            key: (fault_final[key] - fault_baseline[key]
                  if fault_final[key] is not None and fault_baseline[key] is not None
                  else None) for key in counter_values}
        last_fault_values = [item["values"].get("last_fault") for item in diagnostics]
        no_new_faults = (
            counters_valid and counters_monotonic and
            all(delta == 0 for delta in fault_deltas.values()) and
            bool(last_fault_values) and all(value is not None for value in last_fault_values) and
            len(set(last_fault_values)) == 1)

        mapping = self.mapping_starts[0] if self.mapping_starts else None
        mapping_arrival = (mapping.get("arrival_wall_ns")
                           if isinstance(mapping, dict) else None)
        mapping_value = (mapping.get("value")
                         if isinstance(mapping, dict) else None)
        mapping_valid = bool(self.mapping_starts) and all(
            item["valid"] for item in self.mapping_starts)
        stable_ns = int(float(self.config["ready_stable_before_mapping_s"]) * 1.0e9)
        diagnostics_before_mapping = [
            item for item in diagnostics if mapping is not None and
            item["arrival_wall_ns"] <= mapping_arrival]
        mapping_after_stable_ready = (
            mapping_valid and first_ready is not None and
            mapping_arrival is not None and mapping_arrival >= first_ready + stable_ns and
            len(diagnostics_before_mapping) >= int(
                self.config["ready_diagnostic_samples_before_mapping"]) and
            all(item["level"] == 0 and
                self._as_bool(item["values"].get("ready", "false"))
                for item in diagnostics_before_mapping) and
            not any(not value and first_ready <= arrival <= mapping_arrival
                    for value, arrival in self.ready_samples))
        mapping_session_bound = (
            mapping_valid and mapping is not None and
            len(expected_session_ids) == 1 and
            mapping_value == expected_session_ids[0])

        domains = self.timesync_domains
        domain_intervals = [_seconds(right["arrival_wall_ns"] - left["arrival_wall_ns"])
                            for left, right in zip(domains, domains[1:])]
        domain_header_ages = [_seconds(item["arrival_ros_ns"] - item["stamp_ns"])
                              for item in domains]
        source_roles = self.config["timesync_source_roles"]
        domain_keyvalues_unique = bool(domains) and all(
            item["values_well_formed"] and not item["duplicate_value_keys"]
            for item in domains)
        domain_schema = bool(domains) and all(
            item["values_well_formed"] and not item["duplicate_value_keys"] and
            set(item["values"]) == DOMAIN_VALUE_KEYS and
            item["values"].get("schema") == self.config["timesync_domain_schema"]
            for item in domains)
        domain_values = bool(domains) and all(
            item["values"].get("domain_id") == self.config["required_timesync_domain"] and
            item["values"].get("measurement_domain") ==
                self.config["required_timesync_domain"] and
            item["values"].get("d435_clock_source") ==
                self.config["required_d435_clock_source"] and
            item["values"].get("fcu_clock_source") ==
                self.config["required_fcu_clock_source"]
            for item in domains)
        domain_sources = bool(domains) and all(
            _valid_sha256(item["values"].get(field)) and
            _valid_sha256(expected_source_hashes.get(source_roles[field])) and
            item["values"][field] == expected_source_hashes[source_roles[field]]
            for item in domains for field in DOMAIN_SOURCE_FIELDS)
        runtime_identities = [item["publisher_runtime_identity"]
                              for item in domains]
        expected_executable_sha = expected_source_hashes.get(
            source_roles["publisher_executable_sha256"])
        domain_runtime_publisher = bool(domains) and all(
            isinstance(item, dict) and set(item) == {
                "callerid", "pid", "uri", "host", "local_host_gate",
                "executable_path", "executable_sha256"} and
            isinstance(item.get("callerid"), str) and bool(item["callerid"]) and
            isinstance(item.get("pid"), int) and not isinstance(item["pid"], bool) and
            item["pid"] > 0 and item.get("local_host_gate") is True and
            isinstance(item.get("uri"), str) and bool(item["uri"]) and
            isinstance(item.get("host"), str) and bool(item["host"]) and
            isinstance(item.get("executable_path"), str) and
            os.path.isabs(item["executable_path"]) and
            _valid_sha256(item.get("executable_sha256")) and
            item["executable_sha256"] == expected_executable_sha
            for item in runtime_identities)
        domain_runtime_publisher = (
            domain_runtime_publisher and
            len({(item["callerid"], item["pid"], item["uri"],
                  item["executable_path"]) for item in runtime_identities}) == 1)
        domain_resets = [self._counter(item["values"].get("reset_counter"))
                         for item in domains]
        domain_sequences = [self._counter(item["values"].get("measurement_sequence"))
                            for item in domains]
        d435_stamps = [self._counter(item["values"].get("d435_stamp_ns"))
                       for item in domains]
        fcu_stamps = [self._counter(item["values"].get("fcu_stamp_ns"))
                      for item in domains]
        measured_offsets = [_strict_int_text(item["values"].get("measured_offset_ns"))
                            for item in domains]
        domain_session_reset = (
            bool(domains) and len(expected_session_ids) == 1 and
            len(expected_reset_counters) == 1 and
            all(item["values"].get("session_id") == expected_session_ids[0]
                for item in domains) and
            all(value is not None and value == expected_reset_counters[0]
                for value in domain_resets))
        domain_measurements = (
            bool(domains) and
            all(value is not None for value in domain_sequences + d435_stamps +
                fcu_stamps + measured_offsets) and
            all(value > 0 for value in d435_stamps + fcu_stamps) and
            all(right == left + 1 for left, right in
                zip(domain_sequences, domain_sequences[1:])) and
            all(right > left for left, right in zip(d435_stamps, d435_stamps[1:])) and
            all(right > left for left, right in zip(fcu_stamps, fcu_stamps[1:])) and
            all(offset == d435 - fcu for offset, d435, fcu in
                zip(measured_offsets, d435_stamps, fcu_stamps)) and
            all(abs(_seconds(offset)) <= float(self.config["timesync_max_abs_offset_s"])
                for offset in measured_offsets))
        measurement_header_ages = [
            _seconds(item["stamp_ns"] - max(d435, fcu))
            for item, d435, fcu in zip(domains, d435_stamps, fcu_stamps)
            if d435 is not None and fcu is not None]
        domain_measurement_freshness = (
            len(measurement_header_ages) == len(domains) and bool(domains) and
            all(abs(value) <= float(self.config["timesync_measurement_max_age_s"])
                for value in measurement_header_ages))
        domain_runtime = (
            len(domains) >= 2 and all(
                isinstance(item["level"], int) and not isinstance(item["level"], bool) and
                item["level"] == 0 and item["stamp_ns"] > 0 for item in domains) and
            all(right["stamp_ns"] > left["stamp_ns"]
                for left, right in zip(domains, domains[1:])) and
            all(0 < value <= float(self.config["diagnostic_max_gap_s"])
                for value in domain_intervals) and
            all(abs(value) <= float(self.config["diagnostic_max_idle_s"])
                for value in domain_header_ages) and
            0 <= _seconds(now_wall_ns - domains[-1]["arrival_wall_ns"]) <=
                float(self.config["diagnostic_max_idle_s"]))
        domain_gate = (not self.config["require_timesync_domain"] or
                       (domain_schema and domain_values and domain_sources and
                        domain_runtime_publisher and
                        domain_session_reset and domain_measurements and
                        domain_measurement_freshness and domain_runtime))
        domain_not_required = not self.config["require_timesync_domain"]
        gates = {
            "hybrid_output_stream": output_stream_report.get("status") == "PASS",
            "ready_true_observed": first_ready is not None,
            "ready_session_latched_no_false": false_after_ready == 0 and final_ready,
            "diagnostics_continuous_and_fresh": diagnostic_continuous and diagnostic_fresh,
            "diagnostic_header_stamps_monotonic": diagnostic_header_gate,
            "diagnostic_header_latency": diagnostic_header_latency_gate,
            "diagnostic_key_values_unique": diagnostic_keyvalues_unique,
            "diagnostics_ready_and_ok": diagnostic_ok,
            "exact_fault_counter_schema": counter_schema,
            "zero_new_fault_counters": no_new_faults,
            "mapping_start_value_nonempty": mapping_valid,
            "mapping_start_session_identity_bound": mapping_session_bound,
            "mapping_start_after_stable_ready": (
                not self.config["require_mapping_start_after_ready"] or
                mapping_after_stable_ready),
            "clock_domain_key_values_unique": (
                domain_not_required or domain_keyvalues_unique),
            "clock_domain_session_reset_bound": (
                domain_not_required or domain_session_reset),
            "clock_domain_actual_measurements": (
                domain_not_required or domain_measurements),
            "clock_domain_measurement_freshness": (
                domain_not_required or domain_measurement_freshness),
            "clock_domain_publisher_and_source_identity": (
                domain_not_required or
                (domain_sources and domain_runtime_publisher)),
            "clock_domain_runtime_publisher_executable": (
                domain_not_required or domain_runtime_publisher),
            "typed_d435_fcu_timesync_domain": domain_gate}
        return {
            "contract": dict(self.config), "ready_sample_count": len(self.ready_samples),
            "first_ready_arrival_wall_ns": first_ready,
            "false_after_ready_count": false_after_ready, "final_ready": final_ready,
            "diagnostic_sample_count_after_ready": len(diagnostics),
            "diagnostic_samples_before_mapping": len(diagnostics_before_mapping),
            "diagnostic_interval_s": _distribution(intervals),
            "diagnostic_arrival_minus_header_s": _distribution(diagnostic_header_ages),
            "diagnostic_fresh": diagnostic_fresh,
            "fault_counter_baseline": fault_baseline,
            "fault_counter_final": fault_final, "fault_counter_deltas": fault_deltas,
            "last_fault_values": last_fault_values,
            "mapping_start_count": len(self.mapping_starts),
            "first_mapping_start": mapping,
            "timesync_domain_sample_count": len(domains),
            "timesync_domain_interval_s": _distribution(domain_intervals),
            "timesync_domain_arrival_minus_header_s": _distribution(domain_header_ages),
            "timesync_measurement_header_age_s": _distribution(measurement_header_ages),
            "timesync_measurement_sequences": domain_sequences,
            "timesync_d435_stamps_ns": d435_stamps,
            "timesync_fcu_stamps_ns": fcu_stamps,
            "timesync_measured_offsets_ns": measured_offsets,
            "timesync_domain_publisher_runtime_identities": runtime_identities,
            "gates": gates, "status": "PASS" if all(gates.values()) else "FAIL"}


def _deep_merge(base, overlay):
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return overlay
    result = {key: value for key, value in base.items()}
    for key, value in overlay.items():
        result[key] = (_deep_merge(result[key], value)
                       if key in result and isinstance(result[key], dict) and
                       isinstance(value, dict) else value)
    return result


def _exact_value(left, right):
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(
            _exact_value(left[key], right[key]) for key in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(
            _exact_value(a, b) for a, b in zip(left, right))
    return left == right


def _copy_tree(value):
    if isinstance(value, dict):
        return {key: _copy_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_copy_tree(item) for item in value]
    return value


def _path_parts(path):
    return path[1:].split("/") if isinstance(path, str) and path.startswith("/") else []


def _tree_has_path(value, path):
    current = value
    for part in _path_parts(path):
        if not isinstance(current, dict) or part not in current:
            return False
        current = current[part]
    return bool(_path_parts(path))


def _without_allowlisted_paths(value, paths):
    result = _copy_tree(value)
    for path in paths:
        parts = _path_parts(path)
        current = result
        for part in parts[:-1]:
            if not isinstance(current, dict) or part not in current:
                current = None
                break
            current = current[part]
        if isinstance(current, dict) and parts and parts[-1] in current:
            del current[parts[-1]]
    return result


def evaluate_candidate_provenance(config, base_config, overlay_config,
                                  runtime_params, camera_config,
                                  runtime_camera_params, build_manifest,
                                  executable_sha256, library_sha256,
                                  source_file_sha256):
    """Evaluate selected overlay/effective params/native build identity exactly."""
    build = build_manifest if isinstance(build_manifest, dict) else {}
    expected_effective = (_deep_merge(base_config, overlay_config)
                          if isinstance(base_config, dict) and
                          isinstance(overlay_config, dict) else None)
    effective_allowlist = config.get("effective_param_ephemeral_allowlist", [])
    camera_allowlist = config.get("private_camera_param_ephemeral_allowlist", [])
    allowlists_valid = (_valid_allowlist(effective_allowlist) and
                        _valid_allowlist(camera_allowlist))
    allowlists_disjoint = (
        allowlists_valid and isinstance(expected_effective, dict) and
        isinstance(camera_config, dict) and isinstance(runtime_params, dict) and
        isinstance(runtime_camera_params, dict) and
        not any(_tree_has_path(expected_effective, path)
                for path in effective_allowlist) and
        not any(_tree_has_path(camera_config, path) for path in camera_allowlist) and
        all(_tree_has_path(runtime_params, path) for path in effective_allowlist) and
        all(_tree_has_path(runtime_camera_params, path) for path in camera_allowlist))
    normalized_runtime_params = (
        _without_allowlisted_paths(runtime_params, effective_allowlist)
        if allowlists_valid and isinstance(runtime_params, dict) else runtime_params)
    normalized_runtime_camera_params = (
        _without_allowlisted_paths(runtime_camera_params, camera_allowlist)
        if allowlists_valid and isinstance(runtime_camera_params, dict)
        else runtime_camera_params)
    expected_effective_sha256 = canonical_value_sha256(expected_effective)
    raw_runtime_effective_sha256 = canonical_value_sha256(runtime_params)
    runtime_effective_sha256 = canonical_value_sha256(normalized_runtime_params)
    expected_camera_sha256 = canonical_value_sha256(camera_config)
    raw_runtime_camera_sha256 = canonical_value_sha256(runtime_camera_params)
    runtime_camera_sha256 = canonical_value_sha256(normalized_runtime_camera_params)
    build_libraries = build.get("dynamic_libraries")
    build_sources = build.get("source_tree_identity", {}).get("files")
    normalized_build_sources = (
        {path: record.get("sha256") for path, record in build_sources.items()
         if isinstance(record, dict)} if isinstance(build_sources, dict) else {})
    library_sha256 = dict(library_sha256 or {})
    source_file_sha256 = dict(source_file_sha256 or {})
    gates = {
        "base_and_selected_overlay_available": (
            isinstance(base_config, dict) and bool(base_config) and
            isinstance(overlay_config, dict) and bool(overlay_config)),
        "selected_phase_b_contract_exact": (
            isinstance(overlay_config, dict) and
            overlay_config.get("common", {}).get("online_intrinsics_en") is False and
            overlay_config.get("imu", {}).get("acc_cov") == 10.0 and
            overlay_config.get("vio", {}).get("img_point_cov") == 1000.0 and
            overlay_config.get("vio", {}).get("outlier_threshold") == 600.0 and
            overlay_config.get("uav", {}).get("imu_rate_odom") is False),
        "ephemeral_param_allowlists_exact_and_disjoint": allowlists_disjoint,
        "effective_params_exact_deep_merge": (
            config.get("require_exact_effective_params") is True and
            expected_effective is not None and
            _exact_value(normalized_runtime_params, expected_effective)),
        "effective_params_canonical_hash_exact": (
            _valid_sha256(expected_effective_sha256) and
            runtime_effective_sha256 == expected_effective_sha256),
        "static_camera_private_namespace_exact": (
            config.get("runtime_camera_param_namespace") == "/laserMapping" and
            isinstance(camera_config, dict) and bool(camera_config) and
            _exact_value(normalized_runtime_camera_params, camera_config)),
        "static_camera_canonical_hash_exact": (
            _valid_sha256(expected_camera_sha256) and
            runtime_camera_sha256 == expected_camera_sha256),
        "build_manifest_schema": (
            build.get("schema") == config.get("required_build_manifest_schema")),
        "build_derived_from_actual_source_and_binary": (
            build.get("derived_from_actual_isolated_devel_and_source") is True),
        "build_source_tree_identity_consistent": (
            _valid_sha256(build.get("source_tree_sha256")) and
            build.get("source_tree_sha256") ==
                build.get("source_tree_identity", {}).get("tree_sha256") and
            _valid_sha256(build.get("identity_sha256"))),
        "executable_digest": (
            _valid_sha256(executable_sha256) and
            executable_sha256 == build.get("executable_sha256")),
        "dynamic_library_set_and_digests": (
            isinstance(build_libraries, dict) and
            set(build_libraries) == FASTLIVO_LIBRARIES and
            set(library_sha256) == FASTLIVO_LIBRARIES and
            all(_valid_sha256(value) and value == build_libraries.get(name)
                for name, value in library_sha256.items())),
        "source_manifest_all_files_exact": (
            bool(normalized_build_sources) and
            set(source_file_sha256) == set(normalized_build_sources) and
            all(_valid_sha256(value) and
                value == normalized_build_sources.get(path)
                for path, value in source_file_sha256.items())),
    }
    return {
        "contract": dict(config), "expected_effective_params": expected_effective,
        "runtime_effective_params": runtime_params,
        "normalized_runtime_effective_params": normalized_runtime_params,
        "expected_effective_params_sha256": expected_effective_sha256,
        "raw_runtime_effective_params_sha256": raw_runtime_effective_sha256,
        "runtime_effective_params_sha256": runtime_effective_sha256,
        "expected_static_camera_params": camera_config,
        "runtime_private_camera_params": runtime_camera_params,
        "normalized_runtime_private_camera_params": normalized_runtime_camera_params,
        "expected_static_camera_params_sha256": expected_camera_sha256,
        "raw_runtime_private_camera_params_sha256": raw_runtime_camera_sha256,
        "runtime_private_camera_params_sha256": runtime_camera_sha256,
        "phase_b_equivalence_claimed": False,
        "camera_info_true_profile_accepted": False,
        "candidate_scope": (
            "Only the exact Phase-B candidate with online_intrinsics_en=false is accepted. "
            "A live CameraInfo=true profile requires a separate camera receipt and rebaseline "
            "and must not be mixed with this candidate."),
        "build_manifest_schema": build.get("schema"),
        "build_manifest_identity_sha256": build.get("identity_sha256"),
        "executable_sha256": executable_sha256,
        "dynamic_library_sha256": library_sha256,
        "source_file_sha256": source_file_sha256,
        "gates": gates, "status": "PASS" if all(gates.values()) else "FAIL"}
