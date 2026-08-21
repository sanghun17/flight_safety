"""Small append-only, self-hashed evidence helpers.

The helpers deliberately do not update an existing run.  A caller gets one
new directory, writes each artifact once with ``O_EXCL``, and writes one final
receipt whose hash is defined over canonical JSON with the hash field omitted.
This is evidence integrity, not a substitute for signing or immutable storage.
"""

from __future__ import print_function

import datetime
import hashlib
import json
import os
import re
import stat
import uuid


SELF_HASH_FIELD = "receipt_self_sha256"
SELF_HASH_RULE = (
    "sha256(canonical UTF-8 JSON of this document with "
    "receipt_self_sha256 omitted; sort_keys=true,separators=(',',':'))"
)
_RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SOURCE_BUNDLE_SCHEMA = "flight_safety/fcu_shadow_source_bundle/v3"
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def default_run_id(prefix):
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return "%s-%s-%s" % (prefix, stamp, uuid.uuid4().hex)


def canonical_json_bytes(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_source_bundle_manifest(path, required_files, required_external=None):
    """Validate exact role/path/digest bindings in a source bundle manifest.

    Each required value is ``{"path": <logical path>, "sha256": <digest>}``.
    A digest appearing elsewhere in the manifest is deliberately insufficient:
    qualification must bind the digest to both its expected role and path.
    """
    failures = []
    document = None
    if (not isinstance(path, str) or not os.path.isabs(path) or
            not os.path.isfile(path) or os.path.islink(path)):
        failures.append("bundle manifest is not an absolute regular non-link file")
    else:
        try:
            with open(path, "r") as stream:
                document = json.load(stream)
        except (OSError, ValueError):
            failures.append("bundle manifest is not valid JSON")
    files = document.get("files") if isinstance(document, dict) else None
    file_roles = document.get("file_roles") if isinstance(document, dict) else None
    external = document.get("external_sources") if isinstance(document, dict) else None
    if not isinstance(document, dict) or document.get("schema") != SOURCE_BUNDLE_SCHEMA:
        failures.append("bundle manifest schema mismatch")
    if not isinstance(files, dict) or not files:
        failures.append("bundle manifest files mapping is missing")
        files = {}
    if not isinstance(file_roles, dict) or not file_roles:
        failures.append("bundle manifest file_roles mapping is missing")
        file_roles = {}
    if not isinstance(external, dict):
        failures.append("bundle manifest external_sources mapping is missing")
        external = {}
    for relative, digest in files.items():
        if (not isinstance(relative, str) or os.path.isabs(relative) or
                relative == ".." or relative.startswith("../") or
                not isinstance(digest, str) or not _SHA256_RE.match(digest)):
            failures.append("bundle manifest has an invalid file entry")
            break
    for label, record in file_roles.items():
        if (not isinstance(label, str) or not label or not isinstance(record, dict) or
                record.get("path") not in files or
                files.get(record.get("path")) != record.get("sha256")):
            failures.append("bundle manifest has an invalid file role entry")
            break
    for label, record in external.items():
        external_path = record.get("path") if isinstance(record, dict) else None
        external_digest = record.get("sha256") if isinstance(record, dict) else None
        if (not isinstance(label, str) or not label or
                not isinstance(external_path, str) or not external_path or
                os.path.isabs(external_path) or external_path == ".." or
                not isinstance(external_digest, str) or
                not _SHA256_RE.match(external_digest)):
            failures.append("bundle manifest has an invalid external source entry")
            break
    file_gates = {}
    for label, expected in sorted(dict(required_files or {}).items()):
        record = file_roles.get(label)
        expected_path = expected.get("path") if isinstance(expected, dict) else None
        digest = expected.get("sha256") if isinstance(expected, dict) else None
        gate = bool(
            isinstance(record, dict) and record.get("path") == expected_path and
            record.get("sha256") == digest and files.get(expected_path) == digest and
            isinstance(expected_path, str) and not os.path.isabs(expected_path) and
            isinstance(digest, str) and _SHA256_RE.match(digest))
        file_gates[label] = gate
        if not gate:
            failures.append("bundle manifest does not bind required file: %s" % label)
    external_gates = {}
    for label, expected in sorted(dict(required_external or {}).items()):
        record = external.get(label)
        expected_path = expected.get("path") if isinstance(expected, dict) else None
        digest = expected.get("sha256") if isinstance(expected, dict) else None
        gate = bool(
            isinstance(record, dict) and record.get("path") == expected_path and
            record.get("sha256") == digest and isinstance(expected_path, str) and
            isinstance(digest, str) and _SHA256_RE.match(digest))
        external_gates[label] = gate
        if not gate:
            failures.append("bundle manifest does not bind external source: %s" % label)
    return {
        "status": "PASS" if not failures else "FAIL", "failures": failures,
        "path": path, "sha256": sha256_file(path) if document is not None else None,
        "schema": document.get("schema") if isinstance(document, dict) else None,
        "file_gates": file_gates, "external_gates": external_gates,
    }


def with_self_hash(document):
    result = dict(document)
    result.pop(SELF_HASH_FIELD, None)
    result[SELF_HASH_FIELD] = sha256_bytes(canonical_json_bytes(result))
    return result


def self_hash_valid(document):
    expected = document.get(SELF_HASH_FIELD)
    if not isinstance(expected, str):
        return False
    core = dict(document)
    core.pop(SELF_HASH_FIELD, None)
    return expected == sha256_bytes(canonical_json_bytes(core))


class AppendOnlyRun(object):
    """Own one newly-created evidence directory; never overwrite a path."""

    def __init__(self, output_root, run_id):
        raw_output_root = str(output_root)
        if not os.path.isabs(raw_output_root):
            raise ValueError("output_root must be absolute")
        output_root = os.path.abspath(raw_output_root)
        if not _RUN_ID_RE.match(str(run_id)):
            raise ValueError("unsafe run_id: %r" % (run_id,))

        if os.path.lexists(output_root):
            root_stat = os.lstat(output_root)
            if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
                raise ValueError("output_root must be a real directory, not a symlink")
        else:
            os.makedirs(output_root, mode=0o750)

        self.output_root = output_root
        self.run_id = str(run_id)
        self.run_dir = os.path.join(output_root, self.run_id)
        os.mkdir(self.run_dir, 0o750)  # atomic refusal if this run already exists
        self.evidence_dir = os.path.join(self.run_dir, "evidence")
        os.mkdir(self.evidence_dir, 0o750)
        self._artifacts = []

    @property
    def artifacts(self):
        return [dict(item) for item in self._artifacts]

    def _checked_relative(self, relative_path):
        relative_path = str(relative_path)
        if os.path.isabs(relative_path):
            raise ValueError("artifact path must be relative")
        normalized = os.path.normpath(relative_path)
        if normalized == ".." or normalized.startswith("../"):
            raise ValueError("artifact escapes run directory")
        return normalized

    def path(self, relative_path):
        return os.path.join(self.run_dir, self._checked_relative(relative_path))

    def write_bytes(self, relative_path, value, media_type="application/octet-stream"):
        relative_path = self._checked_relative(relative_path)
        path = self.path(relative_path)
        parent = os.path.dirname(path)
        if not os.path.isdir(parent):
            os.makedirs(parent, mode=0o750)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(path, flags, 0o440)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(value)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception:
            try:
                os.close(descriptor)
            except OSError:
                pass
            raise
        return self._record(relative_path, path, media_type)

    def write_text(self, relative_path, value, media_type="text/plain; charset=utf-8"):
        return self.write_bytes(relative_path, str(value).encode("utf-8"), media_type)

    def adopt_existing(self, relative_path, media_type="application/octet-stream"):
        """Hash a command-created file in this fresh run, refusing links/types."""
        relative_path = self._checked_relative(relative_path)
        path = self.path(relative_path)
        file_stat = os.lstat(path)
        if not stat.S_ISREG(file_stat.st_mode):
            raise ValueError("artifact is not a regular file: %s" % relative_path)
        os.chmod(path, 0o440)
        return self._record(relative_path, path, media_type)

    def _record(self, relative_path, path, media_type):
        artifact = {
            "path": relative_path,
            "media_type": media_type,
            "size_bytes": os.path.getsize(path),
            "sha256": sha256_file(path),
        }
        self._artifacts.append(artifact)
        return dict(artifact)

    def finalize(self, document, filename="receipt.json"):
        if os.path.lexists(self.path(filename)):
            raise FileExistsError(self.path(filename))
        result = dict(document)
        result["self_hash_rule"] = SELF_HASH_RULE
        result["artifacts"] = sorted(self.artifacts, key=lambda item: item["path"])
        result = with_self_hash(result)
        payload = json.dumps(
            result, sort_keys=True, indent=2, ensure_ascii=False,
            allow_nan=False).encode("utf-8") + b"\n"
        self.write_bytes(filename, payload, "application/json")
        # The receipt cannot list itself without a circular artifact hash.  Its
        # canonical self hash above is the authoritative receipt integrity rule.
        return self.path(filename), result
