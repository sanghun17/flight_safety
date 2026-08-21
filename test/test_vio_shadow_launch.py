from pathlib import Path
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).parents[1]


def test_shadow_launch_is_isolated_from_mux_response_and_mavros_outputs():
    launch_path = ROOT / "launch" / "vio_shadow_preflight.launch"
    root = ElementTree.parse(str(launch_path)).getroot()
    includes = root.findall("include")
    nodes = root.findall("node")

    assert includes == []
    assert len(nodes) == 1
    assert nodes[0].attrib["type"] == "vio_shadow_preflight.py"
    launch_text = launch_path.read_text()
    assert "/mavros/vision" not in launch_text
    assert "/mavros/odometry/out" not in launch_text
    assert root.findall("remap") == []
    assert nodes[0].findall("remap") == []


def test_shadow_script_contains_subscribers_but_no_explicit_publish_or_service_path():
    script = (ROOT / "scripts" / "vio_shadow_preflight.py").read_text()
    assert "rospy.Subscriber" in script
    assert "rospy.Publisher" not in script
    assert "ServiceProxy" not in script
    assert "mavros_msgs" not in script
    assert "{item.key: item.value" not in script
    assert script.count("[(item.key, item.value) for item in status.values]") == 3


def test_default_profile_exposes_current_covariance_and_identity_blockers():
    profile = yaml.safe_load((ROOT / "config" / "vio_shadow_preflight.yaml").read_text())
    safety = profile["safety_invariants"]
    assert safety == {
        "mode": "shadow_only",
        "allow_ros_publish": False,
        "allow_fcu_actuation": False,
        "fcu_output_topics": [],
    }
    correction = next(item for item in profile["streams"]
                      if item["name"] == "correction_pose")
    assert correction["topic"] == "/aft_mapped_to_body_correction_pose_cov"
    assert correction["message_type"] == "geometry_msgs/PoseWithCovarianceStamped"
    assert correction["min_rate_hz"] == 8.0
    assert correction["require_pose_covariance"] is True
    high_rate = next(item for item in profile["streams"]
                     if item["name"] == "propagated_odometry")
    assert high_rate["min_rate_hz"] >= 30.0
    assert high_rate["require_pose_covariance"] is True
    assert high_rate["require_twist_covariance"] is True
    assert profile["identity"]["status_topic"] == ""
    assert profile["identity"]["status_message_type"] == (
        "diagnostic_msgs/DiagnosticArray")
    hybrid_stream = next(item for item in profile["streams"]
                         if item["name"] == "hybrid_imu_output")
    assert hybrid_stream["topic"] == "/camera/imu_hybrid"
    assert hybrid_stream["message_type"] == "sensor_msgs/Imu"
    assert hybrid_stream["min_rate_hz"] >= 150.0
    assert hybrid_stream["require_imu_payload"] is True
    assert hybrid_stream["require_imu_orientation"] is True
    assert hybrid_stream["require_imu_covariance"] is True
    assert hybrid_stream["covariance_min_diagonal"] > 0.0
    assert hybrid_stream["covariance_min_eigenvalue"] > 0.0
    hybrid = profile["hybrid_imu"]
    assert hybrid["ready_topic"] == "/camera/imu_hybrid/ready"
    assert hybrid["diagnostics_topic"] == "/camera/imu_hybrid/diagnostics"
    assert hybrid["require_mapping_start_after_ready"] is True
    assert hybrid["ready_diagnostic_samples_before_mapping"] >= 2
    assert hybrid["required_fault_counter_keys"]
    assert hybrid["mapping_start_topic"] == ""
    assert hybrid["require_timesync_domain"] is True
    assert hybrid["timesync_domain_message_type"] == (
        "diagnostic_msgs/DiagnosticArray")
    assert hybrid["timesync_domain_topic"] == ""
    assert profile["linkage"]["require_typed_identity_join"] is True
    provenance = profile["producer_provenance"]
    assert provenance["runtime_param_namespace"] == ""
    assert provenance["runtime_camera_param_namespace"] == ""
    assert provenance["require_exact_effective_params"] is True
    assert provenance["effective_param_ephemeral_allowlist"] == []
    assert provenance["private_camera_param_ephemeral_allowlist"] == []
    assert all(value == "" for value in provenance["dynamic_library_paths"].values())
    assert profile["evidence"]["require_source_identity"] is True
    assert profile["evidence"]["source_paths"]["selected_candidate_overlay"] == ""
    assert profile["evidence"]["source_paths"]["fastlivo_build_manifest"] == ""
