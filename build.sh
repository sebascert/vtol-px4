#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case "${1:-sitl}" in
    sitl) config=px4_sitl_vtol; target_dir=sitl ;;
    firmware) config=px4_fmu-v6c_default; target_dir=firmware ;;
    -h|--help)
        echo "Usage: $0 [sitl|firmware] [CMake options...]"
        exit 0
        ;;
    *) echo "Usage: $0 [sitl|firmware] [CMake options...]" >&2; exit 2 ;;
esac
if (( $# )); then shift; fi

python3 -m venv "$PROJECT_DIR/build/venv"
python="$PROJECT_DIR/build/venv/bin/python3"
"$python" -m pip install --no-cache-dir --disable-pip-version-check -r "$PROJECT_DIR/requirements.txt"

cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build/$target_dir" -G Ninja \
    -DCONFIG="$config" -DPYTHON_EXECUTABLE="$python" "$@"
cmake --build "$PROJECT_DIR/build/$target_dir" --parallel 2
