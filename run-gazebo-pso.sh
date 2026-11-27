#!/usr/bin/env bash
# Run the desktop PX4 binary as the external-HIL board (SIL-HIL).
# Usage: run-gazebo-pso.sh --usb DEV --companion DEV [px4 options...]
# DEVs are serial devices or PTYs (e.g. a socat pair); the HIL bridge opens
# the other end of --usb and the allocator the other end of --companion.
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SITL="$PROJECT_DIR/build/sitl-hil"
if [[ ! -f "$SITL/etc/init.d-posix/airframes/22001_stallion_pso_silhil" ]]; then
	echo 'Build the SIL-HIL target first: ./controller/build.sh sitl' >&2
	exit 1
fi
usb=${VTOL_SILHIL_USB_DEV:-}
companion=${VTOL_SILHIL_COMPANION_DEV:-}
run_dir="$PROJECT_DIR/build/gazebo-pso-hil/rootfs"
args=()
while [[ $# -gt 0 ]]; do
	case "$1" in
	--usb)
		usb=$2
		shift 2
		;;
	--companion)
		companion=$2
		shift 2
		;;
	-w)
		run_dir=$2
		shift 2
		;;
	*)
		args+=("$1")
		shift
		;;
	esac
done
if [[ -z "$usb" || -z "$companion" ]]; then
	echo 'Provide --usb and --companion devices (or VTOL_SILHIL_*_DEV).' >&2
	exit 2
fi
for device in "$usb" "$companion"; do
	if [[ ! -e "$device" ]]; then
		echo "Device $device does not exist; start the PTY pairs first." >&2
		exit 1
	fi
done
mkdir -p "$run_dir"
export VTOL_SILHIL_USB_DEV=$usb VTOL_SILHIL_COMPANION_DEV=$companion
export PX4_SYS_AUTOSTART=22001
exec "$SITL/bin/px4" "$SITL/etc" -w "$run_dir" "${args[@]}"
