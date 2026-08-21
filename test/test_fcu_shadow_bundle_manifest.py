import hashlib
import json
from pathlib import Path

from flight_safety.append_only_evidence import (
    SOURCE_BUNDLE_SCHEMA, validate_source_bundle_manifest)


ROOT = Path(__file__).parents[1]
MANIFEST = ROOT / "config" / "fcu_shadow_bundle_manifest.json"


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_bundle_manifest_binds_every_declared_package_and_external_source():
    document = json.loads(MANIFEST.read_text())
    assert document["schema"] == SOURCE_BUNDLE_SCHEMA
    assert document["self_hash_policy"] == (
        "manifest omitted from files to avoid recursion; receipt hashes manifest artifact")
    assert document["files"]
    for relative, expected in document["files"].items():
        path = ROOT / relative
        assert path.is_file(), relative
        assert _sha256(path) == expected, relative
    assert document["file_roles"]
    for label, record in document["file_roles"].items():
        assert record["path"] in document["files"], label
        assert record["sha256"] == document["files"][record["path"]], label
    assert document["external_sources"]
    for label, record in document["external_sources"].items():
        path = (ROOT / record["path"]).resolve()
        assert path.is_file(), label
        assert _sha256(path) == record["sha256"], label


def test_bundle_validator_rejects_digest_under_wrong_path_or_role():
    document = json.loads(MANIFEST.read_text())
    role, record = next(iter(document["file_roles"].items()))
    wrong_path = dict(record, path="wrong/" + record["path"])
    report = validate_source_bundle_manifest(
        str(MANIFEST.resolve()), {role: wrong_path})
    assert report["status"] == "FAIL"
    assert report["file_gates"][role] is False

    external_role, external = next(iter(document["external_sources"].items()))
    wrong_external_path = dict(external, path="wrong/" + external["path"])
    report = validate_source_bundle_manifest(
        str(MANIFEST.resolve()), {}, {external_role: wrong_external_path})
    assert report["status"] == "FAIL"
    assert report["external_gates"][external_role] is False
