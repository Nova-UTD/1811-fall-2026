#!/usr/bin/env bash
# Start the full vehicle stack from the host, in the existing dev container.
set -euo pipefail
cd "$(dirname "$0")"
exec ./scripts/dev.sh bash /vehicle_1811/scripts/full_stack.sh "$@"
