# VTOL26 PX4

Custom PX4 firmware for VTOL26, connecting the flight controller to a PSO
allocator running on a companion computer. PX4 supplies aircraft state and
control targets, and receives actuator allocations over MAVLink.

This repository keeps PX4 versioning and upstream updates separate from the
companion software, vehicle model and simulation in [VTOL26][vtol26].

Clone this project through VTOL26 rather than independently. Follow the
[VTOL26 README][vtol26-readme] to set up the complete workspace.

Follow the [Pixhawk flashing guide][flashing-guide] to build, install and verify
the firmware.

## Build

Choose a target explicitly:

- **`sitl`** builds PX4 for desktop simulation and companion communication
  tests.
- **`firmware`** builds the Pixhawk 6C firmware package to flash with
  QGroundControl.

Run either command from the VTOL26 workspace root:

```sh
./controller/build.sh sitl --vehicle-config config/vehicle.json
./controller/build.sh firmware --vehicle-config config/vehicle.json
```

Both targets require `--vehicle-config` to select the vehicle configuration.

## Changes to PX4

- **Allocation reception:** receives PSO results from the companion over
  MAVLink. It validates the sender identifiers, message format and numeric
  values before publishing accepted results within PX4.
- **Allocation reporting:** publishes the `pso_allocation` topic with nine
  actuator values, solver fitness, position error, convergence status and
  receipt timestamps for monitoring the companion's results.
- **Telemetry setup:** configures TELEM2 for Onboard telemetry at 57600 baud
  without hardware flow control. The settings are applied at every boot, with
  USB retained for QGroundControl and unused serial MAVLink instances disabled.
- **Simulation and testing:** adds a desktop communication profile using PX4's
  built-in simulator and enables HIL output support in the Pixhawk build.
  Protocol tests exercise the companion's serial sender and PX4's allocation
  validator, including rejection of invalid packets.
- **Build workflow:** provides firmware and desktop build targets, manages
  Python dependencies, and validates and records the selected vehicle
  configuration alongside each build.

## Dependencies

- Git
- CMake
- Ninja
- C/C++ compiler and Make
- pkg-config
- json-c development headers for the communication tests
- Python 3.12 with pip and virtual environment support
- ARM bare-metal GCC toolchain, binutils, Newlib and C++ libraries
- Bison
- Flex
- genromfs
- gperf

`build.sh` creates a virtual environment and installs the Python packages from
[requirements.txt](requirements.txt) automatically.

## Working on Ubuntu

Ubuntu is recommended: the project scripts use Bash, and development and testing
have been done on Ubuntu.

On **Ubuntu 24.04**, install the build tools and ARM toolchain:

```sh
sudo apt update
sudo apt install --no-install-recommends \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  libjson-c-dev \
  git \
  python3 \
  python3-venv \
  python3-pip \
  bison \
  flex \
  genromfs \
  gperf \
  gcc-arm-none-eabi \
  binutils-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib
```

## Testing

After building, run these commands from the VTOL26 workspace root:

```sh
controller/build/venv/bin/python3 -m unittest discover \
  -s controller/vtol_tests -p 'test_*.py'
```

This checks the build script and vehicle configuration validation.

To test allocation and MAVLink communication:

```sh
cmake -S controller/vtol_tests/communication -B controller/build/host-tests \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON &&
cmake --build controller/build/host-tests --parallel 2 &&
ctest --test-dir controller/build/host-tests --output-on-failure
```

These tests use virtual serial connections and require no connected hardware.

[vtol26]: https://github.com/sebascert/vtol26
[vtol26-readme]: https://github.com/sebascert/vtol26/blob/main/README.md
[flashing-guide]: https://github.com/sebascert/vtol26/blob/main/docs/guides/flash_pixhawk.md
