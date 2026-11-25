#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

usage() {
	echo "Usage: $0 <sitl|firmware> [CMake options...]"
}

case "${1:-}" in
sitl | firmware)
	target_dir=$1
	shift
	;;
-h | --help)
	usage
	exit 0
	;;
*)
	usage >&2
	exit 2
	;;
esac

for argument in "$@"; do
	case "$argument" in
	-h | --help)
		usage
		exit 0
		;;
	--vehicle-config*)
		echo 'Vehicle configuration is fixed at ~/.local/share/vtol26/config/vehicle.json; run config/install.sh.' >&2
		exit 2
		;;
	esac
done

case "$target_dir" in
sitl)
	config=px4_sitl_vtol
	export GZ_DISTRO=harmonic
	;;
firmware)
	config=px4_fmu-v6c_default
	;;
esac

python3 -m venv "$PROJECT_DIR/build/venv"
python="$PROJECT_DIR/build/venv/bin/python3"
"$python" -m pip install --no-cache-dir --disable-pip-version-check \
	-r "$PROJECT_DIR/requirements.txt"

"$HOME/.local/share/vtol26/config/venv/bin/python3" -I "$HOME/.local/share/vtol26/config/validate_vehicle.py" \
	--output-dir "$PROJECT_DIR/build/$target_dir"

cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build/$target_dir" -G Ninja \
	-DCONFIG="$config" \
	-DPYTHON_EXECUTABLE="$python" \
	"$@"

cmake --build "$PROJECT_DIR/build/$target_dir" --parallel 2
