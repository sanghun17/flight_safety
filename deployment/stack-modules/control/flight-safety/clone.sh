#!/bin/bash
# Fetch flight_safety into its own workspace (ws/flight-safety). It builds standalone
# using only dependencies declared by this module.
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/flight-safety/src/flight_safety"
REPO="${SAFETY_REPO:-git@github.com:sanghun17/flight_safety.git}"
BRANCH="${SAFETY_REVISION:-53a12aeb398a3d9a3b5bff0e8b47405643f4c307}"
bash "$ROOT/scripts/lib/clone_repo.sh" "$DST" "$REPO" "$BRANCH"
