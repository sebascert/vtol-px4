#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

usage() {
    echo "Usage: $0 <sitl|firmware> --vehicle-config PATH [CMake options...]"
}

case "${1:-}" in
    sitl|firmware)
        target_dir=$1
        shift
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

vehicle_config=""
cmake_args=()
while (( $# )); do
    case "$1" in
        --vehicle-config)
            if (( $# < 2 )) || [[ -n "$vehicle_config" ]]; then
                usage >&2
                exit 2
            fi

            vehicle_config=$2
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            cmake_args+=("$1")
            shift
            ;;
    esac
done

if [[ -z "$vehicle_config" || ! -f "$vehicle_config" || ! -r "$vehicle_config" ]]; then
    echo "Provide a readable vehicle file with --vehicle-config PATH." >&2
    exit 2
fi

case "$target_dir" in
    sitl)
        config=px4_sitl_vtol
        ;;
    firmware)
        config=px4_fmu-v6c_default
        ;;
esac

python3 -m venv "$PROJECT_DIR/build/venv"
python="$PROJECT_DIR/build/venv/bin/python3"
"$python" -m pip install --no-cache-dir --disable-pip-version-check \
    -r "$PROJECT_DIR/requirements.txt"

"$python" "$PROJECT_DIR/vehicle_config.py" "$vehicle_config" \
    --output-dir "$PROJECT_DIR/build/$target_dir"

cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build/$target_dir" -G Ninja \
    -DCONFIG="$config" \
    -DPYTHON_EXECUTABLE="$python" \
    "${cmake_args[@]}"

cmake --build "$PROJECT_DIR/build/$target_dir" --parallel 2
