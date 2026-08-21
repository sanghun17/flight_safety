#!/usr/bin/env python3
"""CLI for one fail-closed, read-only FCU audit receipt."""

from __future__ import print_function

import argparse
import json
import os
import sys

from flight_safety.fcu_audit_receipt import FcuAuditCollector, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Capture immutable read-only MAVROS/FCU preflight evidence")
    parser.add_argument("--config", required=True, help="audit YAML")
    parser.add_argument("--output-root", required=True, help="absolute append-only root")
    parser.add_argument("--run-id", help="optional unique run identifier")
    args = parser.parse_args(argv)

    try:
        config = load_config(args.config)
        config["_executed_collector_entrypoint"] = os.path.realpath(__file__)
        collector = FcuAuditCollector(config)
        receipt_path, receipt = collector.collect(args.output_root, args.run_id)
    except Exception as exc:
        print("FCU audit could not materialize a receipt: %s" % exc, file=sys.stderr)
        return 3

    print(json.dumps({
        "receipt": receipt_path,
        "status": receipt["status"],
        "receipt_self_sha256": receipt["receipt_self_sha256"],
    }, sort_keys=True))
    return 0 if receipt["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
