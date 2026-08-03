"""Reproducibility sidecar for one arm-triggered flight recording.

The arm recorder owns the session boundary, so it is also the only component
that can reliably say which runtime/configuration belonged to a bag.  This
module keeps that work out of the safety callback: the start snapshot runs in a
background thread and the final snapshot is completed before the recorder
drops its ``.ready`` sync marker.

The output is ordinary YAML, deliberately self-contained.  It includes the
ROS parameter tree (the effective runtime view), source YAML contents and
hashes, git state/diffs, model artifact hashes, ROS graph, and the start/end
MAVROS states.  Capture errors are data in the manifest; they never prevent a
bag from being recorded or synchronized.
"""

from __future__ import print_function

import copy
import datetime
import hashlib
import json
import os
import platform
import socket
import subprocess
import threading

import yaml


SCHEMA_VERSION = 1
DEFAULT_SUFFIX = ".runtime_manifest.yaml"


def _utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _plain(value):
    """Convert values to the scalar/list/mapping subset safe_dump understands."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return str(value)


def _fingerprint(value):
    encoded = json.dumps(
        _plain(value), sort_keys=True, separators=(",", ":"),
        ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


class RuntimeManifestRecorder(object):
    """Capture and atomically maintain ``<bag>.runtime_manifest.yaml``."""

    def __init__(self, enabled, repo_paths, config_paths,
                 command_timeout_s=6.0, suffix=DEFAULT_SUFFIX,
                 git_diff_max_bytes=262144, log_info=None, log_warn=None,
                 command_runner=None):
        self.enabled = bool(enabled)
        self.repo_paths = dict(repo_paths or {})
        self.config_paths = dict(config_paths or {})
        self.command_timeout_s = float(command_timeout_s)
        self.suffix = str(suffix)
        self.git_diff_max_bytes = int(git_diff_max_bytes)
        self.log_info = log_info or (lambda *args: None)
        self.log_warn = log_warn or (lambda *args: None)
        self.command_runner = command_runner
        self._lock = threading.Lock()
        self._thread = None
        self._path = None
        self._document = None
        self._hash_cache = {}

    def start(self, bag_path, mavros_state, rosbag_command):
        if not self.enabled:
            return None
        self._path = self.path_for_bag(bag_path)
        self._document = {
            "schema": "flight_safety/runtime_manifest",
            "schema_version": SCHEMA_VERSION,
            "capture_status": "capturing_start",
            "session": {
                "bag_path_at_arm": bag_path,
                "bag_path_final": None,
                "manifest_path": self._path,
                "arm_wall_time_utc": _utc_now(),
                "disarm_wall_time_utc": None,
            },
            "recorder": {
                "rosbag_command": list(rosbag_command),
            },
            "snapshots": {},
            "comparison": {},
            "capture_errors": [],
        }
        self._write_atomic(self._path, self._document)
        self._thread = threading.Thread(
            target=self._capture_start, args=(_plain(mavros_state),))
        self._thread.daemon = True
        self._thread.start()
        self.log_info("[arm_recorder] runtime manifest start capture -> %s", self._path)
        return self._path

    def finish(self, original_bag_path, final_bag_path, mavros_state,
               rosbag_returncode=None, webcam_path=None):
        """Finish the YAML before ``.ready`` is created, then return its path."""
        if not self.enabled or self._document is None:
            return None
        if self._thread is not None:
            # Every subprocess in _capture_snapshot has a timeout.  Joining here
            # guarantees the synced YAML cannot race a late start-snapshot write.
            self._thread.join()
            self._thread = None

        end_snapshot = self._capture_snapshot("end", _plain(mavros_state))
        bag_artifact = self._capture_bag_artifact(final_bag_path)
        end_path = self.path_for_bag(final_bag_path)
        with self._lock:
            self._document["snapshots"]["end"] = end_snapshot
            session = self._document["session"]
            session["bag_path_final"] = final_bag_path
            session["manifest_path"] = end_path
            session["disarm_wall_time_utc"] = _utc_now()
            self._document["recorder"].update({
                "rosbag_returncode": rosbag_returncode,
                "webcam_path": webcam_path,
                "bag_path_before_webcam_rename": original_bag_path,
                "bag_artifact": bag_artifact,
            })
            self._document["comparison"] = self._compare_snapshots(
                self._document["snapshots"].get("start"), end_snapshot)
            self._document["capture_status"] = (
                "complete_with_errors" if self._document["capture_errors"]
                else "complete")
            document = copy.deepcopy(self._document)

        self._write_atomic(end_path, document)
        old_path = self._path
        if old_path != end_path:
            try:
                os.unlink(old_path)
            except OSError:
                pass
        self._path = end_path
        self.log_info("[arm_recorder] runtime manifest finalized -> %s", end_path)
        return end_path

    def path_for_bag(self, bag_path):
        if bag_path.endswith(".active"):
            bag_path = bag_path[:-len(".active")]
        base, extension = os.path.splitext(bag_path)
        if extension == ".bag":
            return base + self.suffix
        return bag_path + self.suffix

    def _capture_start(self, mavros_state):
        snapshot = self._capture_snapshot("start", mavros_state)
        with self._lock:
            self._document["snapshots"]["start"] = snapshot
            self._document["capture_status"] = "recording"
            document = copy.deepcopy(self._document)
            path = self._path
        self._write_atomic(path, document)

    def _capture_snapshot(self, phase, mavros_state):
        snapshot = {
            "phase": phase,
            "captured_at_utc": _utc_now(),
            "host": self._capture_host(),
            "mavros_state": mavros_state,
            "ros": self._capture_ros(),
            "repositories": {},
            "config_files": {},
        }
        for name, path in sorted(self.repo_paths.items()):
            snapshot["repositories"][name] = self._capture_git(path)
        for name, path in sorted(self.config_paths.items()):
            snapshot["config_files"][name] = self._inspect_file(path, parse_yaml=True)
        snapshot["model_artifacts"] = self._capture_model_artifacts(
            snapshot["config_files"].get("planning"))
        snapshot["fingerprint_sha256"] = _fingerprint(snapshot)
        return snapshot

    def _capture_host(self):
        env_names = (
            "ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME", "ROS_DISTRO",
            "RISK_AWARE_CHECKPOINTS", "CUDA_VISIBLE_DEVICES",
        )
        tegra_path = "/etc/nv_tegra_release"
        host = {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "environment": {name: os.environ.get(name) for name in env_names},
        }
        if os.path.exists(tegra_path):
            try:
                with open(tegra_path, "r") as stream:
                    host["nv_tegra_release"] = stream.read().strip()
            except OSError as exc:
                host["nv_tegra_release_error"] = str(exc)
        return host

    def _capture_ros(self):
        params_result = self._run(["rosparam", "get", "/"])
        parameters = None
        if params_result["ok"]:
            try:
                parameters = yaml.safe_load(params_result["stdout"]) or {}
                # The parsed tree below is the authoritative copy; retaining
                # rosparam's textual YAML would nearly double the sidecar.
                params_result["stdout"] = ""
            except Exception as exc:
                params_result["parse_error"] = str(exc)
                self._error("rosparam YAML parse failed: %s" % exc)
        nodes = self._line_command(["rosnode", "list"])
        topics = self._line_command(["rostopic", "list"])
        services = self._line_command(["rosservice", "list"])
        return {
            "parameters": _plain(parameters),
            "parameters_sha256": _fingerprint(parameters),
            "rosparam_command": params_result,
            "nodes": nodes,
            "topics": topics,
            "services": services,
        }

    def _line_command(self, command):
        result = self._run(command)
        return {
            "ok": result["ok"],
            "values": sorted(line for line in result["stdout"].splitlines() if line),
            "returncode": result["returncode"],
            "stderr": result["stderr"],
            "error": result.get("error"),
        }

    def _capture_git(self, path):
        result = {"path": path, "exists": os.path.isdir(path)}
        if not result["exists"]:
            result["error"] = "repository path does not exist"
            return result
        inside = self._git(path, ["rev-parse", "--is-inside-work-tree"])
        if not inside["ok"] or inside["stdout"].strip() != "true":
            result["error"] = "not a git worktree"
            result["probe"] = inside
            return result

        commands = {
            "root": ["rev-parse", "--show-toplevel"],
            "head": ["rev-parse", "HEAD"],
            "branch": ["branch", "--show-current"],
            "describe": ["describe", "--always", "--dirty", "--tags"],
            "origin": ["remote", "get-url", "origin"],
            # "normal" records untracked directory names without recursively
            # walking caches/build trees during an ARM session.
            "status_porcelain": ["status", "--short", "--untracked-files=normal"],
        }
        for key, args in commands.items():
            command_result = self._git(path, args)
            result[key] = command_result["stdout"].strip() if command_result["ok"] else None
            if not command_result["ok"]:
                result[key + "_error"] = command_result.get("error") or command_result["stderr"]

        worktree_diff = self._git(path, ["diff", "--no-ext-diff", "--binary"])
        index_diff = self._git(path, ["diff", "--cached", "--no-ext-diff", "--binary"])
        result["dirty"] = bool(result.get("status_porcelain"))
        result["worktree_diff"] = self._bounded_text(worktree_diff)
        result["index_diff"] = self._bounded_text(index_diff)
        return result

    def _git(self, path, args):
        return self._run(["git", "-C", path] + list(args))

    def _bounded_text(self, command_result):
        text = command_result["stdout"]
        raw = text.encode("utf-8", errors="replace")
        truncated = len(raw) > self.git_diff_max_bytes
        shown = raw[:self.git_diff_max_bytes].decode("utf-8", errors="replace")
        return {
            "ok": command_result["ok"],
            "sha256": hashlib.sha256(raw).hexdigest(),
            "bytes": len(raw),
            "truncated": truncated,
            "content": shown,
            "returncode": command_result["returncode"],
            "stderr": command_result["stderr"],
            "error": command_result.get("error"),
        }

    def _inspect_file(self, path, parse_yaml):
        expanded = os.path.expanduser(path)
        result = {"path": path, "resolved_path": expanded, "exists": os.path.isfile(expanded)}
        if not result["exists"]:
            return result
        try:
            stat = os.stat(expanded)
            result.update({
                "size_bytes": stat.st_size,
                "mtime_utc": datetime.datetime.fromtimestamp(
                    stat.st_mtime, datetime.timezone.utc).isoformat(),
                "sha256": self._cached_sha256(expanded, stat),
            })
            if parse_yaml:
                with open(expanded, "r") as stream:
                    result["content"] = _plain(yaml.safe_load(stream))
        except Exception as exc:
            result["error"] = str(exc)
            self._error("file capture failed for %s: %s" % (expanded, exc))
        return result

    def _cached_sha256(self, path, stat):
        key = (path, stat.st_size, stat.st_mtime_ns)
        if key not in self._hash_cache:
            self._hash_cache[key] = _file_sha256(path)
        return self._hash_cache[key]

    def _capture_model_artifacts(self, planning_file):
        result = {}
        content = (planning_file or {}).get("content")
        ete = content.get("ete_net") if isinstance(content, dict) else None
        if not isinstance(ete, dict):
            result["error"] = "planning config/ete_net unavailable"
            return result
        env_name = ete.get("checkpoint_env_var")
        fallback = os.path.expanduser(str(ete.get("checkpoint_default_dir", "")))
        env_value = os.environ.get(env_name) if env_name else None
        base = env_value if env_value is not None else fallback
        result.update({
            "checkpoint_base_source": "environment" if env_value is not None else "planning_default",
            "checkpoint_environment_variable": env_name,
            "checkpoint_environment_value": env_value,
            "checkpoint_base_resolved": base,
            "checkpoint_expected_sha256": ete.get("checkpoint_sha256"),
        })
        for key, rel_key in (
                ("checkpoint", "checkpoint_relative_path"),
                ("context_stats", "context_stats_relative_path")):
            relative = ete.get(rel_key)
            if relative is None:
                result[key] = {"exists": False, "error": "missing %s" % rel_key}
                continue
            result[key] = self._inspect_file(os.path.join(base, relative), parse_yaml=False)
        actual = result.get("checkpoint", {}).get("sha256")
        expected = result.get("checkpoint_expected_sha256")
        result["checkpoint_hash_matches_config"] = bool(actual and expected and actual == expected)
        return result

    def _capture_bag_artifact(self, path):
        result = self._inspect_file(path, parse_yaml=False)
        info = self._run(["rosbag", "info", "--yaml", path])
        result["rosbag_info_command"] = info
        if info["ok"]:
            try:
                result["rosbag_info"] = _plain(yaml.safe_load(info["stdout"]) or {})
                # Keep the parsed value; the raw YAML duplicates it and can be
                # very large for bags with many connection records.
                result["rosbag_info_command"]["stdout"] = ""
            except Exception as exc:
                result["rosbag_info_parse_error"] = str(exc)
                self._error("rosbag info YAML parse failed: %s" % exc)
        return result

    def _compare_snapshots(self, start, end):
        if not start or not end:
            return {"available": False}
        def pick(snapshot, *keys):
            value = snapshot
            for key in keys:
                value = value.get(key, {}) if isinstance(value, dict) else {}
            return value
        return {
            "available": True,
            "start_fingerprint_sha256": start.get("fingerprint_sha256"),
            "end_fingerprint_sha256": end.get("fingerprint_sha256"),
            "ros_parameters_changed": (
                pick(start, "ros", "parameters_sha256") !=
                pick(end, "ros", "parameters_sha256")),
            "repository_heads_changed": {
                name: pick(start, "repositories", name, "head") !=
                      pick(end, "repositories", name, "head")
                for name in sorted(set(start.get("repositories", {})) |
                                   set(end.get("repositories", {})))
            },
            "config_sha256_changed": {
                name: pick(start, "config_files", name, "sha256") !=
                      pick(end, "config_files", name, "sha256")
                for name in sorted(set(start.get("config_files", {})) |
                                   set(end.get("config_files", {})))
            },
        }

    def _run(self, command):
        if self.command_runner is not None:
            return self.command_runner(command, self.command_timeout_s)
        try:
            proc = subprocess.run(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                timeout=self.command_timeout_s, check=False)
            stdout = proc.stdout.decode("utf-8", errors="replace")
            stderr = proc.stderr.decode("utf-8", errors="replace").strip()
            return {
                "ok": proc.returncode == 0,
                "returncode": proc.returncode,
                "stdout": stdout,
                "stderr": stderr,
                "error": None,
            }
        except Exception as exc:
            self._error("command failed (%s): %s" % (" ".join(command), exc))
            return {
                "ok": False, "returncode": None, "stdout": "", "stderr": "",
                "error": str(exc),
            }

    def _error(self, message):
        self.log_warn("[arm_recorder] runtime manifest: %s", message)
        with self._lock:
            if self._document is not None:
                self._document["capture_errors"].append({
                    "time_utc": _utc_now(), "message": message,
                })

    @staticmethod
    def _write_atomic(path, document):
        directory = os.path.dirname(path) or "."
        if not os.path.isdir(directory):
            os.makedirs(directory)
        temporary = path + ".tmp.%d" % os.getpid()
        with open(temporary, "w") as stream:
            yaml.safe_dump(
                _plain(document), stream, default_flow_style=False,
                sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
