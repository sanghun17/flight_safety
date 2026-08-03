import hashlib
import os

import yaml

from flight_safety.runtime_manifest import RuntimeManifestRecorder


def _runner(command, _timeout):
    if command[:3] == ["rosparam", "get", "/"]:
        stdout = "planning_config:\n  local_ours:\n    safety_weight_x: 0.2\n"
    elif command[:2] == ["rosnode", "list"]:
        stdout = "/jax_planner\n/flight_safety_recorder\n"
    elif command[:2] == ["rostopic", "list"]:
        stdout = "/mavros/state\n/jax/optimal_trajectory\n"
    elif command[:2] == ["rosservice", "list"]:
        stdout = "/recorder/start\n/recorder/stop\n"
    elif command[:3] == ["rosbag", "info", "--yaml"]:
        stdout = "duration: 1.5\nmessages: 42\n"
    else:
        return {
            "ok": False, "returncode": 127, "stdout": "", "stderr": "unknown",
            "error": "unknown command",
        }
    return {"ok": True, "returncode": 0, "stdout": stdout, "stderr": "", "error": None}


def test_manifest_is_renamed_finalized_and_contains_runtime_provenance(tmp_path, monkeypatch):
    assets = tmp_path / "assets"
    checkpoint = assets / "model" / "best_val.pth"
    context = assets / "stats.pt"
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")
    context.write_bytes(b"context")
    expected = hashlib.sha256(b"checkpoint").hexdigest()
    monkeypatch.setenv("TEST_CHECKPOINT_ROOT", str(assets))

    planning = tmp_path / "planning.yaml"
    planning.write_text(yaml.safe_dump({
        "ete_net": {
            "checkpoint_env_var": "TEST_CHECKPOINT_ROOT",
            "checkpoint_default_dir": "/unused",
            "checkpoint_relative_path": "model/best_val.pth",
            "checkpoint_sha256": expected,
            "context_stats_relative_path": "stats.pt",
        }
    }))
    recorder_cfg = tmp_path / "recorder.yaml"
    recorder_cfg.write_text("record_all: true\n")

    initial_bag = tmp_path / "safety_2026-08-04-00-00-00.bag"
    final_bag = tmp_path / "flight_2026-08-04_00-00-00.bag"
    final_bag.write_bytes(b"bag")
    manifest = RuntimeManifestRecorder(
        enabled=True,
        repo_paths={"missing": str(tmp_path / "no-repo")},
        config_paths={"planning": str(planning), "recorder": str(recorder_cfg)},
        command_runner=_runner)

    arm = {"armed": True, "mode": "POSCTL"}
    disarm = {"armed": False, "mode": "POSCTL"}
    start_path = manifest.start(str(initial_bag), arm, ["rosbag", "record", "-a"])
    final_path = manifest.finish(
        str(initial_bag), str(final_bag), disarm,
        rosbag_returncode=0, webcam_path="/recordings/flight.mp4")

    assert not os.path.exists(start_path)
    assert final_path == str(final_bag).replace(".bag", ".runtime_manifest.yaml")
    document = yaml.safe_load(open(final_path))
    assert document["capture_status"] == "complete"
    assert document["session"]["bag_path_final"] == str(final_bag)
    assert document["snapshots"]["start"]["mavros_state"]["armed"] is True
    assert document["snapshots"]["end"]["mavros_state"]["armed"] is False
    assert document["snapshots"]["start"]["ros"]["parameters"]["planning_config"]["local_ours"]["safety_weight_x"] == 0.2
    model = document["snapshots"]["start"]["model_artifacts"]
    assert model["checkpoint"]["sha256"] == expected
    assert model["checkpoint_hash_matches_config"] is True
    assert document["comparison"]["ros_parameters_changed"] is False
    assert document["recorder"]["bag_artifact"]["size_bytes"] == 3
    assert document["recorder"]["bag_artifact"]["rosbag_info"]["messages"] == 42


def test_disabled_manifest_is_a_noop(tmp_path):
    manifest = RuntimeManifestRecorder(False, {}, {}, command_runner=_runner)
    bag = str(tmp_path / "flight.bag")
    assert manifest.start(bag, {}, ["rosbag", "record"]) is None
    assert manifest.finish(bag, bag, {}) is None
