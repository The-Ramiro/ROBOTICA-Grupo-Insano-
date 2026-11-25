#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export QT_QPA_PLATFORM=xcb
exec /usr/bin/python3 "$ROOT/editar_limites.py"
