from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from flight_safety.vio_shadow import (
    HybridImuAccumulator, IdentityAccumulator, LinkageAccumulator,
    PRODUCER_IDENTITY_VALUE_KEYS, StreamAccumulator, evaluate_candidate_provenance,
    normal_duration_completion, validate_profile)


ROOT = Path(__file__).parents[1]
PRODUCER_IDENTITY = {
    key: format(index + 1, "x") * 64
    for index, key in enumerate(sorted(PRODUCER_IDENTITY_VALUE_KEYS))}


def _covariance():
    values = [0.0] * 36
    for index in range(6):
        values[index * 6 + index] = 0.01
    return values


def _stream_config():
    return {
        "name": "candidate", "topic": "/vio/shadow",
        "message_type": "nav_msgs/Odometry", "queue_size": 1000,
        "expected_frame_id": "odom", "expected_child_frame_id": "base_link",
        "min_samples": 50, "min_rate_hz": 30.0, "max_gap_s": 0.05,
        "max_idle_s": 0.1, "max_abs_header_latency_s": 0.1,
        "max_first_sample_delay_s": 0.05, "min_window_coverage_ratio": 0.95,
        "covariance_symmetry_tolerance": 1.0e-9,
        "covariance_psd_tolerance": 1.0e-9,
        "covariance_min_diagonal": 1.0e-12,
        "covariance_min_eigenvalue": 1.0e-12,
        "covariance_max_diagonal": 1000.0, "covariance_max_abs": 1000.0,
        "quaternion_norm_tolerance": 1.0e-3,
        "require_pose_covariance": True, "require_twist_covariance": True,
        "require_pose_payload": True, "require_twist_payload": True,
        "require_imu_payload": False, "require_imu_orientation": False,
        "require_imu_covariance": False,
    }


def _observe_good(stream, stamp_ns, arrival_wall_ns=None):
    stream.observe(
        stamp_ns, stamp_ns + 10_000_000, "odom", "base_link",
        _covariance(), _covariance(),
        stamp_ns + 10_000_000 if arrival_wall_ns is None else arrival_wall_ns,
        [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0],
        [0.1, 0.2, 0.3], [0.01, 0.02, 0.03])


def test_exact_stream_metrics_payload_psd_bounds_duration_and_coverage_pass():
    start_ns = 1_700_000_000_000_000_000
    stream = StreamAccumulator("candidate", _stream_config(), start_ns)
    for index in range(100):
        _observe_good(stream, start_ns + index * 20_000_000)
    report = stream.summarize(
        start_ns + 99 * 20_000_000 + 10_000_000,
        start_ns + 99 * 20_000_000 + 10_000_000, 1.98)

    assert report["status"] == "PASS"
    assert abs(report["arrival_rate_hz"] - 50.0) < 1.0e-9
    assert report["window_coverage_ratio"] == 1.0
    assert report["gates"]["first_sample_delay"] is True
    assert report["payload"]["pose"]["valid"] == 100
    assert report["payload"]["twist"]["valid"] == 100
    assert report["covariance"]["pose"]["psd"] == 100
    assert report["covariance"]["twist"]["within_bounds"] == 100
    assert len(stream.raw_records) == 100


def test_nonfinite_payload_bad_quaternion_indefinite_covariance_and_gap_fail():
    config = _stream_config()
    config["min_samples"] = 2
    stream = StreamAccumulator("candidate", config, 10_000_000_000)
    bad_covariance = _covariance()
    bad_covariance[1] = bad_covariance[6] = 0.02  # eigenvalue -0.01
    stream.observe(
        1_700_000_000_000_000_000, 1_700_000_000_001_000_000,
        "wrong", "base_link", bad_covariance, bad_covariance, 10_010_000_000,
        [float("nan"), 0, 0], [0, 0, 0, 2], [0, 0, 0], [0, 0, 0])
    stream.observe(
        1_700_000_000_100_000_000, 1_700_000_000_101_000_000,
        "wrong", "base_link", bad_covariance, bad_covariance, 10_110_000_000,
        [0, 0, 0], [0, 0, 0, 2], [0, 0, 0], [0, 0, 0])
    report = stream.summarize(
        10_110_000_000, 1_700_000_000_101_000_000, 0.1)
    assert report["status"] == "FAIL"
    assert report["gap_count"] == 1
    assert report["frame_mismatch_count"] == 2
    assert report["gates"]["pose_payload"] is False
    assert report["gates"]["pose_covariance"] is False
    assert report["covariance"]["pose"]["psd"] == 0
    assert stream.raw_records[0]["pose_position"][0] == "NaN"


def test_hybrid_imu_output_payload_must_be_finite():
    config = _stream_config()
    config.update({
        "name": "hybrid", "topic": "/camera/imu_hybrid",
        "message_type": "sensor_msgs/Imu", "expected_frame_id": "camera",
        "expected_child_frame_id": "", "min_samples": 2, "min_rate_hz": 100.0,
        "max_gap_s": 0.02, "require_pose_covariance": False,
        "require_twist_covariance": False, "require_pose_payload": False,
        "require_twist_payload": False, "require_imu_payload": True,
        "require_imu_orientation": True, "require_imu_covariance": True})
    start = 1_700_000_000_000_000_000
    covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    stream = StreamAccumulator("hybrid", config, start)
    for index in range(2):
        stamp = start + index * 5_000_000
        stream.observe(
            stamp, stamp + 1_000_000, "camera", "",
            arrival_wall_ns=stamp + 1_000_000,
            imu_angular_velocity=[0.1, 0.2, 0.3],
            imu_linear_acceleration=[1.0, 2.0, 9.8],
            imu_orientation=[0.0, 0.0, 0.0, 1.0],
            imu_orientation_covariance=covariance,
            imu_angular_velocity_covariance=covariance,
            imu_linear_acceleration_covariance=covariance)
    report = stream.summarize(start + 6_000_000, start + 6_000_000, 0.005)
    assert report["status"] == "PASS"
    assert report["payload"]["imu"]["valid"] == 2
    assert report["gates"]["imu_covariance"] is True

    bad = StreamAccumulator("hybrid", config, start)
    for index in range(2):
        stamp = start + index * 5_000_000
        bad.observe(
            stamp, stamp + 1_000_000, "camera", "",
            arrival_wall_ns=stamp + 1_000_000,
            imu_angular_velocity=[float("nan"), 0.2, 0.3],
            imu_linear_acceleration=[1.0, 2.0, 9.8],
            imu_orientation=[0.0, 0.0, 0.0, 1.0],
            imu_orientation_covariance=[-1.0] + [0.0] * 8,
            imu_angular_velocity_covariance=covariance,
            imu_linear_acceleration_covariance=covariance)
    bad_report = bad.summarize(
        start + 6_000_000, start + 6_000_000, 0.005)
    assert bad_report["gates"]["imu_payload"] is False
    assert bad_report["gates"]["imu_covariance"] is False
    assert bad_report["covariance"]["imu"]["orientation"]["known"] == 0

    for rejected_covariance in (
            [0.0] * 9,
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0],
            [0.01, 0.01, 0.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.01]):
        rejected = StreamAccumulator("hybrid", config, start)
        for index in range(2):
            stamp = start + index * 5_000_000
            rejected.observe(
                stamp, stamp + 1_000_000, "camera", "",
                arrival_wall_ns=stamp + 1_000_000,
                imu_angular_velocity=[0.1, 0.2, 0.3],
                imu_linear_acceleration=[1.0, 2.0, 9.8],
                imu_orientation=[0.0, 0.0, 0.0, 1.0],
                imu_orientation_covariance=rejected_covariance,
                imu_angular_velocity_covariance=covariance,
                imu_linear_acceleration_covariance=covariance)
        rejected_report = rejected.summarize(
            start + 6_000_000, start + 6_000_000, 0.005)
        assert rejected_report["status"] == "FAIL"
        assert rejected_report["gates"]["imu_covariance"] is False


def _identity_config():
    return {
        "status_topic": "/vio/identity_join",
        "status_message_type": "diagnostic_msgs/DiagnosticArray",
        "status_name": "fast_livo/identity_join",
        "status_schema": "flight_safety/fastlivo_identity_join/v1",
        "min_samples": 2, "max_idle_s": 0.5,
        "max_abs_header_latency_s": 0.1}


def _observe_identity(identity, sequence, correction_stamp_ns, arrival_wall_ns,
                      session="session-a", reset=0):
    values = {
        "schema": "flight_safety/fastlivo_identity_join/v1",
        "session_id": session, "reset_counter": str(reset),
        "correction_sequence": str(sequence),
        "correction_stamp_ns": str(correction_stamp_ns)}
    values.update(PRODUCER_IDENTITY)
    identity.observe_join(
        "fast_livo/identity_join", 0, "joined",
        values,
        correction_stamp_ns, arrival_wall_ns, correction_stamp_ns + 1_000_000)


def test_identity_typed_join_is_explicit_fresh_and_reset_change_fails():
    identity = IdentityAccumulator(_identity_config(), PRODUCER_IDENTITY)
    assert identity.summarize(1_000_000_000)["status"] == "FAIL"
    _observe_identity(identity, 7, 1_000_000_000, 10_000_000_000)
    _observe_identity(identity, 8, 1_100_000_000, 10_100_000_000)
    assert identity.summarize(10_200_000_000)["status"] == "PASS"
    assert identity.summarize(11_000_000_000)["gates"]["fresh_at_finish"] is False
    _observe_identity(identity, 9, 1_200_000_000, 11_000_000_000, reset=1)
    reset = identity.summarize(11_100_000_000)
    assert reset["status"] == "FAIL"
    assert reset["gates"]["no_reset_during_window"] is False

    wrong_expected = dict(PRODUCER_IDENTITY)
    wrong_expected["executable_sha256"] = "f" * 64
    mismatched = IdentityAccumulator(_identity_config(), wrong_expected)
    _observe_identity(mismatched, 1, 2_000_000_000, 12_000_000_000)
    _observe_identity(mismatched, 2, 2_100_000_000, 12_100_000_000)
    mismatch_report = mismatched.summarize(12_200_000_000)
    assert mismatch_report["status"] == "FAIL"
    assert mismatch_report["gates"]["typed_exact_schema"] is False


def test_identity_duplicate_keyvalue_is_preserved_and_rejected():
    identity = IdentityAccumulator(_identity_config(), PRODUCER_IDENTITY)
    values = [
        ("schema", "flight_safety/fastlivo_identity_join/v1"),
        ("session_id", "session-a"), ("reset_counter", "0"),
        ("correction_sequence", "1"), ("correction_stamp_ns", "1000000000")]
    values.extend(sorted(PRODUCER_IDENTITY.items()))
    values.append(("session_id", "session-shadow"))
    identity.observe_join(
        "fast_livo/identity_join", 0, "joined", values,
        1_000_000_000, 10_000_000_000, 1_001_000_000)
    report = identity.summarize(10_100_000_000)
    assert report["status"] == "FAIL"
    assert report["gates"]["no_duplicate_key_values"] is False
    assert identity.raw_records[0]["duplicate_value_keys"] == ["session_id"]
    assert identity.raw_records[0]["values_raw"][-1] == {
        "key": "session_id", "value": "session-shadow"}


def test_correction_propagated_and_typed_identity_one_to_one_linkage():
    config = {
        "correction_stream": "correction", "propagated_stream": "propagated",
        "max_header_delta_s": 0.02, "max_status_arrival_delta_s": 0.05,
        "max_identity_correction_stamp_delta_s": 0.001,
        "require_typed_identity_join": True}
    linkage = LinkageAccumulator(config)
    identity = IdentityAccumulator(_identity_config(), PRODUCER_IDENTITY)
    for index in range(3):
        stamp = 1_000_000_000 + index * 100_000_000
        arrival = 10_000_000_000 + index * 100_000_000
        linkage.observe_stream("correction", stamp, arrival)
        linkage.observe_stream("propagated", stamp + 1_000_000, arrival)
        _observe_identity(identity, index + 7, stamp, arrival + 2_000_000)
    assert linkage.summarize(identity.join_records)["status"] == "PASS"
    missing = LinkageAccumulator(config)
    missing.observe_stream("correction", 1_000_000_000, 10_000_000_000)
    missing.observe_stream("propagated", 1_001_000_000, 10_000_000_000)
    report = missing.summarize([])
    assert report["status"] == "FAIL"
    assert report["gates"]["one_typed_identity_per_correction"] is False


def test_hybrid_ready_diagnostics_fault_latch_mapping_and_clock_domain():
    config = {
        "output_stream": "hybrid", "ready_topic": "/camera/imu_hybrid/ready",
        "diagnostics_topic": "/camera/imu_hybrid/diagnostics",
        "diagnostic_name": "d435i_tools/hybrid_imu",
        "diagnostic_max_gap_s": 0.25, "diagnostic_max_idle_s": 0.25,
        "mapping_start_topic": "/vio/mapping_start",
        "mapping_start_message_type": "std_msgs/String",
        "require_mapping_start_after_ready": True,
        "ready_stable_before_mapping_s": 0.05,
        "ready_diagnostic_samples_before_mapping": 2,
        "required_fault_counter_keys": ["drop_wait_timeout"],
        "timesync_domain_topic": "/camera/imu_hybrid/timesync_domain",
        "timesync_domain_message_type": "diagnostic_msgs/DiagnosticArray",
        "timesync_domain_status_name": "d435i_tools/clock_domain",
        "timesync_domain_schema": "flight_safety/d435_fcu_clock_domain/v1",
        "required_timesync_domain": "COMMON_ROS_TIME_VERIFIED",
        "required_d435_clock_source": "D435_HEADER_ROS_TIME",
        "required_fcu_clock_source": "FCU_MAVROS_ROS_TIME",
        "timesync_measurement_max_age_s": 0.05,
        "timesync_max_abs_offset_s": 0.05,
        "timesync_source_roles": {
            "hybrid_imu_node_sha256": "hybrid_imu_node",
            "hybrid_imu_core_sha256": "hybrid_imu_core",
            "d435_launch_sha256": "hybrid_imu_launch",
            "mavros_timesync_plugin_sha256": "mavros_timesync_plugin",
            "publisher_executable_sha256": "clock_domain_publisher_executable",
            "publisher_source_sha256": "clock_domain_publisher_source",
            "publisher_build_manifest_sha256":
                "clock_domain_publisher_build_manifest"},
        "require_timesync_domain": True}
    hybrid = HybridImuAccumulator(config)
    start = 10_000_000_000
    hybrid.observe_ready(True, start)
    values = {"ready": "True", "drop_wait_timeout": "2", "last_fault": "drop_wait_timeout"}
    hybrid.observe_diagnostic(
        "d435i_tools/hybrid_imu", 0, "healthy", values,
        start, start, start + 1_000_000)
    hybrid.observe_diagnostic(
        "d435i_tools/hybrid_imu", 0, "healthy", values,
        start + 100_000_000, start + 100_000_000, start + 101_000_000)
    hybrid.observe_mapping_start("session-a", start + 120_000_000)
    hashes = {
        "hybrid_imu_node": "1" * 64, "hybrid_imu_core": "2" * 64,
        "hybrid_imu_launch": "3" * 64, "mavros_timesync_plugin": "4" * 64,
        "clock_domain_publisher_executable": "5" * 64,
        "clock_domain_publisher_source": "6" * 64,
        "clock_domain_publisher_build_manifest": "7" * 64}
    publisher_runtime = {
        "callerid": "/qualified_clock_domain", "pid": 4242,
        "uri": "http://localhost:1234/", "host": "localhost",
        "local_host_gate": True,
        "executable_path": "/qualified/clock_domain_publisher",
        "executable_sha256": hashes["clock_domain_publisher_executable"]}
    def domain(sequence, d435_stamp, fcu_stamp):
        return {
            "schema": "flight_safety/d435_fcu_clock_domain/v1",
            "session_id": "session-a", "reset_counter": "0",
            "measurement_sequence": str(sequence),
            "domain_id": "COMMON_ROS_TIME_VERIFIED",
            "measurement_domain": "COMMON_ROS_TIME_VERIFIED",
            "d435_clock_source": "D435_HEADER_ROS_TIME",
            "fcu_clock_source": "FCU_MAVROS_ROS_TIME",
            "d435_stamp_ns": str(d435_stamp), "fcu_stamp_ns": str(fcu_stamp),
            "measured_offset_ns": str(d435_stamp - fcu_stamp),
            "hybrid_imu_node_sha256": hashes["hybrid_imu_node"],
            "hybrid_imu_core_sha256": hashes["hybrid_imu_core"],
            "d435_launch_sha256": hashes["hybrid_imu_launch"],
            "mavros_timesync_plugin_sha256": hashes["mavros_timesync_plugin"],
            "publisher_executable_sha256": hashes[
                "clock_domain_publisher_executable"],
            "publisher_source_sha256": hashes["clock_domain_publisher_source"],
            "publisher_build_manifest_sha256": hashes[
                "clock_domain_publisher_build_manifest"]}
    domain_one = domain(10, start + 19_000_000, start + 18_000_000)
    domain_two = domain(11, start + 109_000_000, start + 108_000_000)
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", domain_one,
        start + 20_000_000, start + 20_000_000, start + 21_000_000,
        publisher_runtime)
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", domain_two,
        start + 110_000_000, start + 110_000_000, start + 111_000_000,
        publisher_runtime)
    output = {"status": "PASS"}
    report = hybrid.summarize(
        start + 130_000_000, output, hashes, ["session-a"], [0])
    assert report["status"] == "PASS"
    assert report["gates"]["clock_domain_runtime_publisher_executable"] is True
    assert report["fault_counter_deltas"]["drop_wait_timeout"] == 0

    wrong_runtime = dict(publisher_runtime, executable_sha256="0" * 64)
    runtime_spoof = domain(12, start + 119_000_000, start + 118_000_000)
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", runtime_spoof,
        start + 120_000_000, start + 120_000_000, start + 121_000_000,
        wrong_runtime)
    runtime_spoof_report = hybrid.summarize(
        start + 130_000_000, output, hashes, ["session-a"], [0])
    assert runtime_spoof_report["status"] == "FAIL"
    assert runtime_spoof_report["gates"][
        "clock_domain_runtime_publisher_executable"] is False

    wrong_domain = dict(
        domain(13, start + 124_000_000, start + 123_000_000),
        publisher_build_manifest_sha256="0" * 64)
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", wrong_domain,
        start + 125_000_000, start + 125_000_000, start + 126_000_000,
        publisher_runtime)
    source_mismatch = hybrid.summarize(
        start + 130_000_000, output, hashes, ["session-a"], [0])
    assert source_mismatch["status"] == "FAIL"
    assert source_mismatch["gates"]["typed_d435_fcu_timesync_domain"] is False

    hybrid.observe_diagnostic(
        "d435i_tools/hybrid_imu", 0, "healthy",
        dict(values, drop_wait_timeout="3"), start + 200_000_000,
        start + 200_000_000, start + 201_000_000)
    fault = hybrid.summarize(
        start + 210_000_000, output, hashes, ["session-a"], [0])
    assert fault["status"] == "FAIL"
    assert fault["gates"]["zero_new_fault_counters"] is False

    duplicate_diagnostic = list(values.items()) + [("ready", "True")]
    hybrid.observe_diagnostic(
        "d435i_tools/hybrid_imu", 0, "healthy", duplicate_diagnostic,
        start + 220_000_000, start + 220_000_000, start + 221_000_000)
    duplicate_diagnostic_report = hybrid.summarize(
        start + 225_000_000, output, hashes, ["session-a"], [0])
    assert duplicate_diagnostic_report["gates"]["diagnostic_key_values_unique"] is False
    assert hybrid.raw_records[-1]["duplicate_value_keys"] == ["ready"]

    duplicate_domain = list(
        domain(14, start + 229_000_000, start + 228_000_000).items())
    duplicate_domain.append(("session_id", "session-a"))
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", duplicate_domain,
        start + 230_000_000, start + 230_000_000, start + 231_000_000,
        publisher_runtime)
    duplicate_domain_report = hybrid.summarize(
        start + 235_000_000, output, hashes, ["session-a"], [0])
    assert duplicate_domain_report["gates"]["clock_domain_key_values_unique"] is False
    assert hybrid.raw_records[-1]["duplicate_value_keys"] == ["session_id"]

    unbound_measurement = domain(15, start + 239_000_000, start + 238_000_000)
    unbound_measurement["reset_counter"] = "1"
    unbound_measurement["measured_offset_ns"] = "999"
    hybrid.observe_timesync_domain(
        "d435i_tools/clock_domain", 0, "bound", unbound_measurement,
        start + 240_000_000, start + 240_000_000, start + 241_000_000,
        publisher_runtime)
    unbound_report = hybrid.summarize(
        start + 245_000_000, output, hashes, ["session-a"], [0])
    assert unbound_report["gates"]["clock_domain_session_reset_bound"] is False
    assert unbound_report["gates"]["clock_domain_actual_measurements"] is False


def test_hybrid_empty_mapping_missing_counter_and_bad_domain_hash_fail_closed():
    profile = yaml.safe_load((ROOT / "config" / "vio_shadow_preflight.yaml").read_text())
    config = deepcopy(profile["hybrid_imu"])
    config.update({
        "mapping_start_topic": "/mapping/start",
        "timesync_domain_topic": "/clock/domain",
        "ready_stable_before_mapping_s": 0.01,
        "ready_diagnostic_samples_before_mapping": 2,
        "required_fault_counter_keys": ["drop_wait_timeout"]})
    hybrid = HybridImuAccumulator(config)
    start = 10_000_000_000
    hybrid.observe_ready(True, start)
    missing = {"ready": "True", "last_fault": "none"}
    for index in range(2):
        hybrid.observe_diagnostic(
            config["diagnostic_name"], 0, "ok", missing,
            start + index * 10_000_000, start + index * 10_000_000,
            start + index * 10_000_000 + 1_000_000)
    hybrid.observe_mapping_start("", start + 30_000_000)
    report = hybrid.summarize(start + 40_000_000, {"status": "PASS"}, {})
    assert report["status"] == "FAIL"
    assert report["gates"]["mapping_start_value_nonempty"] is False
    assert report["gates"]["exact_fault_counter_schema"] is False
    assert report["gates"]["typed_d435_fcu_timesync_domain"] is False


def test_candidate_provenance_rejects_live_overlay_mismatch():
    config = {
        "runtime_param_namespace": "/", "runtime_camera_param_namespace": "/laserMapping",
        "executable_path": "/bin/x",
        "dynamic_library_paths": {name: "/lib/" + name for name in (
            "libimu_proc.so", "liblaser_mapping.so", "liblio.so",
            "libpre.so", "libvio.so")},
        "source_root": "/src", "required_build_manifest_schema": "build/v1",
        "require_exact_effective_params": True,
        "effective_param_ephemeral_allowlist": [],
        "private_camera_param_ephemeral_allowlist": []}
    base = {"vio": {"outlier_threshold": 1000.0, "img_point_cov": 100.0},
            "imu": {"acc_cov": 0.1}, "uav": {"imu_rate_odom": True},
            "common": {"img_en": 1, "online_intrinsics_en": True}}
    overlay = {
        "common": {"online_intrinsics_en": False}, "imu": {"acc_cov": 10.0},
        "vio": {"img_point_cov": 1000.0, "outlier_threshold": 600.0},
        "uav": {"imu_rate_odom": False}}
    runtime = {
        "vio": {"outlier_threshold": 600.0, "img_point_cov": 1000.0},
        "imu": {"acc_cov": 10.0}, "uav": {"imu_rate_odom": False},
        "common": {"img_en": 1, "online_intrinsics_en": False}}
    libraries = {name: str(index + 1) * 64 for index, name in enumerate(sorted(
        config["dynamic_library_paths"]))}
    sources = {"src/main.cpp": "a" * 64}
    camera = {"cam_model": "Pinhole", "cam_fx": 609.4}
    runtime_camera = {"cam_model": "Pinhole", "cam_fx": 609.4}
    build = {
        "schema": "build/v1", "derived_from_actual_isolated_devel_and_source": True,
        "identity_sha256": "e" * 64, "source_tree_sha256": "d" * 64,
        "executable_sha256": "f" * 64, "dynamic_libraries": libraries,
        "source_tree_identity": {"tree_sha256": "d" * 64, "files": {
            "src/main.cpp": {"sha256": "a" * 64}}}}
    passed = evaluate_candidate_provenance(
        config, base, overlay, runtime, camera, runtime_camera, build,
        "f" * 64, libraries, sources)
    assert passed["status"] == "PASS"
    assert passed["gates"]["selected_phase_b_contract_exact"] is True
    assert passed["expected_effective_params_sha256"] == \
        passed["runtime_effective_params_sha256"]
    live_mismatch = deepcopy(runtime)
    live_mismatch["vio"]["outlier_threshold"] = 1000.0
    live_mismatch["uav"]["imu_rate_odom"] = True
    failed = evaluate_candidate_provenance(
        config, base, overlay, live_mismatch, camera, runtime_camera, build,
        "f" * 64, libraries, sources)
    assert failed["status"] == "FAIL"
    assert failed["gates"]["effective_params_exact_deep_merge"] is False
    bad_camera = evaluate_candidate_provenance(
        config, base, overlay, runtime, camera,
        {"cam_model": "Pinhole", "cam_fx": 500.0}, build,
        "f" * 64, libraries, sources)
    assert bad_camera["status"] == "FAIL"
    assert bad_camera["gates"]["static_camera_private_namespace_exact"] is False

    effective_extra = dict(runtime, unexpected_runtime_param=True)
    extra_effective_report = evaluate_candidate_provenance(
        config, base, overlay, effective_extra, camera, runtime_camera, build,
        "f" * 64, libraries, sources)
    assert extra_effective_report["status"] == "FAIL"
    assert extra_effective_report["gates"]["effective_params_exact_deep_merge"] is False
    camera_extra = dict(runtime_camera, unrelated_private_param=True)
    extra_camera_report = evaluate_candidate_provenance(
        config, base, overlay, runtime, camera, camera_extra, build,
        "f" * 64, libraries, sources)
    assert extra_camera_report["status"] == "FAIL"
    assert extra_camera_report["gates"]["static_camera_private_namespace_exact"] is False

    allowlisted = deepcopy(config)
    allowlisted["effective_param_ephemeral_allowlist"] = ["/unexpected_runtime_param"]
    allowlisted["private_camera_param_ephemeral_allowlist"] = [
        "/unrelated_private_param"]
    explicit_ephemeral = evaluate_candidate_provenance(
        allowlisted, base, overlay, effective_extra, camera, camera_extra, build,
        "f" * 64, libraries, sources)
    assert explicit_ephemeral["status"] == "PASS"
    assert explicit_ephemeral["expected_effective_params_sha256"] == \
        explicit_ephemeral["runtime_effective_params_sha256"]

    masks_required = deepcopy(config)
    masks_required["effective_param_ephemeral_allowlist"] = [
        "/vio/outlier_threshold"]
    masked_required_report = evaluate_candidate_provenance(
        masks_required, base, overlay, runtime, camera, runtime_camera, build,
        "f" * 64, libraries, sources)
    assert masked_required_report["status"] == "FAIL"
    assert masked_required_report["gates"][
        "ephemeral_param_allowlists_exact_and_disjoint"] is False


def test_profile_schema_rejects_unknown_keys_and_default_blockers_are_explicit():
    profile = yaml.safe_load((ROOT / "config" / "vio_shadow_preflight.yaml").read_text())
    assert validate_profile(profile, 60.0, "/tmp/shadow") is True
    unknown = deepcopy(profile)
    unknown["streams"][0]["silently_ignored_typo"] = True
    with pytest.raises(ValueError, match="unknown keys"):
        validate_profile(unknown, 60.0, "/tmp/shadow")
    assert profile["identity"]["status_topic"] == ""
    assert profile["identity"]["status_message_type"] == "diagnostic_msgs/DiagnosticArray"
    assert profile["linkage"]["require_typed_identity_join"] is True
    assert profile["hybrid_imu"]["mapping_start_topic"] == ""
    assert profile["hybrid_imu"]["timesync_domain_topic"] == ""
    assert profile["evidence"]["source_paths"]["bundle_manifest"] == ""
    scalar = deepcopy(profile)
    scalar["identity"]["status_message_type"] = "std_msgs/UInt32"
    with pytest.raises(ValueError, match="DiagnosticArray"):
        validate_profile(scalar, 60.0, "/tmp/shadow")
    string_domain = deepcopy(profile)
    string_domain["hybrid_imu"]["timesync_domain_message_type"] = "std_msgs/String"
    with pytest.raises(ValueError, match="DiagnosticArray"):
        validate_profile(string_domain, 60.0, "/tmp/shadow")
    zero_covariance_floor = deepcopy(profile)
    zero_covariance_floor["streams"][2]["covariance_min_diagonal"] = 0.0
    with pytest.raises(ValueError, match="strictly positive"):
        validate_profile(zero_covariance_floor, 60.0, "/tmp/shadow")
    wildcard_allowlist = deepcopy(profile)
    wildcard_allowlist["producer_provenance"][
        "effective_param_ephemeral_allowlist"] = ["/roslaunch/*"]
    with pytest.raises(ValueError, match="without wildcards"):
        validate_profile(wildcard_allowlist, 60.0, "/tmp/shadow")


def test_normal_duration_gate_rejects_shutdown_early_and_excess_overrun():
    assert normal_duration_completion("duration_complete", 60.01, 60.0, 1.0)
    assert not normal_duration_completion("ros_shutdown", 60.01, 60.0, 1.0)
    assert not normal_duration_completion("duration_complete", 59.99, 60.0, 1.0)
    assert not normal_duration_completion("duration_complete", 61.01, 60.0, 1.0)
