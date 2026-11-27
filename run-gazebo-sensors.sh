#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SITL="$PROJECT_DIR/build/sitl"
if [[ ! -f "$SITL/etc/init.d-posix/airframes/4500_gz_stallion_sensors" ]]; then
	echo 'Build controller sitl-native first; see controller/README_VTOL.md.' >&2
	exit 1
fi
run_dir="$PROJECT_DIR/build/gazebo-sensors/rootfs"
mkdir -p "$run_dir"
export PX4_SYS_AUTOSTART=4500 PX4_SIMULATOR=gz PX4_SIM_MODEL=stallion_vtol
export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=pvtol_world PX4_GZ_MODEL_NAME=controlled_drone
export GZ_DISTRO=harmonic
exec "$SITL/bin/px4" "$SITL/etc" -w "$run_dir" "$@"
