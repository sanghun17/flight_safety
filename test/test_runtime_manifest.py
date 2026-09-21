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


def test_config_snapshot_preserves_shell_and_launch_text(tmp_path):
    env = tmp_path / "runtime.env"
    text = '# configuration\nexport SPEED=0.5\nexport KEEP="${KEEP:-true}"\n'
    env.write_text(text)
    launch = tmp_path / "trial.launch"
    launch.write_text('<launch>\n  <arg name="speed" default="0.5"/>\n</launch>\n')
    config = tmp_path / "controller.yaml"
    config.write_text('speed: 0.5\n')
    recorder = RuntimeManifestRecorder(True, {}, {
        'environment': str(env), 'launch': str(launch), 'controller': str(config)
    }, command_runner=_runner)
    snapshot = recorder._capture_snapshot('start', {})
    assert snapshot['config_files']['environment']['content'] == text
    assert snapshot['config_files']['launch']['content'] == launch.read_text()
    assert snapshot['config_files']['controller']['content'] == {'speed': .5}
    assert all('error' not in f for f in snapshot['config_files'].values())
    assert snapshot['model_artifacts'] == {'applicable': False}


def test_invalid_yaml_still_reports_capture_error(tmp_path):
    config = tmp_path / 'broken.yaml'
    config.write_text('speed: [\n')
    recorder = RuntimeManifestRecorder(True, {}, {}, command_runner=_runner)
    assert 'error' in recorder._inspect_file(str(config), parse_yaml=True)


def test_git_trust_is_scoped_to_configured_repository(tmp_path):
    commands = []
    def runner(command, timeout):
        commands.append(command)
        return dict(ok=True, returncode=0, stdout='true\n', stderr='', error=None)
    recorder = RuntimeManifestRecorder(True, {'component': str(tmp_path)}, {}, command_runner=runner)
    recorder._git(str(tmp_path), ['rev-parse', 'HEAD'])
    assert commands == [['git', '-c', 'safe.directory=' + str(tmp_path.resolve()),
                         '-C', str(tmp_path.resolve()), 'rev-parse', 'HEAD']]


def test_git_snapshot_records_real_commit_and_dirty_state(tmp_path):
    import subprocess
    def git(*args):
        return subprocess.check_output(['git', '-C', str(tmp_path)] + list(args), text=True).strip()
    git('init')
    config = tmp_path / 'config.yaml'
    config.write_text('speed: 0.5\n')
    git('add', 'config.yaml')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-m', 'Initial config')
    recorder = RuntimeManifestRecorder(True, {'component': str(tmp_path)}, {})
    snapshot = recorder._capture_git(str(tmp_path))
    assert snapshot['head'] == git('rev-parse', 'HEAD')
    assert snapshot['dirty'] is False
    config.write_text('speed: 2.0\n')
    snapshot = recorder._capture_git(str(tmp_path))
    assert snapshot['dirty'] is True
    assert '+speed: 2.0' in snapshot['worktree_diff']['content']
