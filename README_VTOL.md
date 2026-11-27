# VTOL26 PX4

Custom PX4 firmware for VTOL26, connecting the flight controller to a PSO
allocator running on a companion computer. PX4 supplies aircraft state and
control targets, and receives actuator allocations over MAVLink.

This repository keeps PX4 versioning and upstream updates separate from the
companion software, vehicle model and simulation in [VTOL26][vtol26].

Clone this project through VTOL26 rather than independently. Follow the
instructions in [VTOL26 README][vtol26-readme] to set up the complete workspace.

Follow the [Pixhawk flashing guide][flashing-guide] to build, install and verify
the firmware.

## Build

Choose a target explicitly:

- **`sitl`** builds the wall-clock PX4 binary for the shared Python SIL/HIL
  bridge in `build/sitl-hil/`.
- **`sitl-native`** builds PX4 for the X500 and profile 4500 native Gazebo
  diagnostics in `build/sitl/`.
- **`firmware`** builds the Pixhawk 6C image. Its default airframe is Stallion
  HIL (22001). A saved airframe selection takes precedence.

Run the chosen command from the VTOL26 workspace root:

```sh
./controller/build.sh sitl
./controller/build.sh firmware
```

Install the [vehicle configuration][vehicle-config] before building.

## Changes to PX4

- **Allocation reception:** receives PSO results from the companion over
  MAVLink. It validates the sender identifiers, message format and numeric
  values before publishing accepted results within PX4.
- **Allocation reporting:** publishes the `pso_allocation` topic with nine
  actuator values, solver fitness, position error, convergence status and
  receipt timestamps for monitoring the companion's results.
- **Native Gazebo actuation:** the legacy profile 4501 uses `pso_control` to
  gate allocations on arming, Offboard mode, fresh inputs, matching targets and
  configured limits. It publishes three motor and six servo controls to the
  custom Gazebo plant. The stock position controller supplies the attitude
  reference; the stock actuator allocator and attitude/rate controllers do not
  run in this profile.
- **SIL and external HIL actuation:** profile 22001 enables the same consumer on
  desktop PX4 and the Pixhawk 6C with `SYS_HITL=1`. Its nine virtual outputs map
  to the configured HIL channels. USB carries HIL sensors/outputs; the Onboard
  companion link excludes HIL traffic and runs at 230400 baud with 50 Hz
  state/target streams. This profile maps the custom stream directly from
  `pso_actuator_outputs`; stock `HIL_ACT_FUNC*` assignments do not define its
  channel mapping.
- **Telemetry setup:** the board configures TELEM2 for Onboard telemetry without
  hardware flow control. The HIL airframe selects 230400 baud and uses USB for
  the HIL bridge. Airframes using the base board settings use 57600 baud.
- **Simulation and testing:** the `sitl-native` target supports PX4's built-in
  SIH simulator and native Gazebo Harmonic bridge/plugins. The Pixhawk build
  includes HIL output support. Protocol tests exercise the companion's serial
  sender and PX4's allocation validator, including rejection of invalid packets.
- **Build workflow:** provides firmware and desktop build targets, manages
  Python dependencies, and validates and records the installed vehicle
  configuration alongside each build. It generates the actuator limits consumed
  by the SITL and HIL PSO controller. Physical motor/servo pin outputs are not
  enabled by the HIL profile.

## Dependencies

- Git
- CMake
- Ninja
- C/C++ compiler and Make
- pkg-config
- json-c development headers for the communication tests
- socat for live allocator communication tests
- Python 3.12 with pip and virtual environment support
- Gazebo Harmonic development libraries for desktop SITL
- OpenCV development headers (`libopencv-dev`) for desktop SITL
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
  libopencv-dev \
  socat \
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

This checks the build script. Vehicle schema, validation and installation checks
live in the parent project's `config/tests/`:

```sh
"$HOME/.local/share/vtol26/config/venv/bin/python3" -I \
  -m unittest discover -s config/tests -v
```

To test allocation and MAVLink communication:

```sh
cmake -S controller/vtol_tests/communication -B controller/build/host-tests \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON &&
cmake --build controller/build/host-tests --parallel 2 &&
ctest --test-dir controller/build/host-tests --output-on-failure
```

These tests use virtual serial connections and require no connected hardware.

For the native X500 communication diagnostic, build `sitl-native` and the
allocator and run from the VTOL26 workspace root:

```sh
controller/build/venv/bin/python3 controller/vtol_tests/gazebo_communication.py
```

It runs the stock X500, checks incoming state/targets and matching allocation
reception, then lands and stops its processes. The allocations remain
diagnostic. See the [Gazebo test guide][gazebo-test] for commands and evidence
paths.

For the complete custom-aircraft feedback loop, build `sitl`, `simulation/` and
the allocator, then run:

```sh
controller/build/venv/bin/python3 controller/vtol_tests/gazebo_closed_loop.py
```

The test connects the Python HIL bridge and allocator to desktop PX4 on two
separate pseudo-terminal pairs. Each loss scenario starts a fresh Gazebo world
and checks movement against Gazebo pose. Results and logs are under
`controller/build/gazebo-closed-loop/`. The
[hardware HIL guide](../docs/guides/hil_simulation.md) covers the physical USB
and companion links.

## Custom aircraft sensors

Build `sitl-native` and `simulation/`, then run:

```sh
python3 controller/vtol_tests/gazebo_sensors.py
```

This checks the custom Stallion's sensor delivery and EKF estimates while
disarmed. Logs and samples are in `controller/build/gazebo-sensors/`. See the
[sensor guide](../docs/gen/test_stallion_sensors.md) for interactive commands.
Profile 4500 has no flight controllers or actuator mappings.

## Installed vehicle contract

Install the shared vehicle files from the VTOL26 root with `./config/install.sh`
before building (dependencies: Bash/coreutils, Python 3 and venv and pip
support). The builder reads `~/.local/share/vtol26/config/vehicle.json` through
the single installed validator
`~/.local/share/vtol26/config/validate_vehicle.py` and its adjacent
`vehicle.schema.json`. There is no vehicle-path option or repository fallback.
Run `./controller/build.sh firmware` or `./controller/build.sh sitl`; each build
records a validated snapshot and compiles its actuator values into PX4.
Rebuild/reflash after changing those values. See the
[configuration README][vehicle-config] for installation and migration details.

[vtol26]: https://github.com/sebascert/vtol26
[vtol26-readme]: https://github.com/sebascert/vtol26/blob/main/README.md
[vehicle-config]: https://github.com/sebascert/vtol26/blob/main/config/README.md
[flashing-guide]: https://github.com/sebascert/vtol26/blob/main/docs/guides/flash_pixhawk.md
[gazebo-test]: https://github.com/sebascert/vtol26/blob/main/docs/gen/test_allocator_gazebo.md

Validator calls use `~/.local/share/vtol26/config/venv/bin/python3`, created by
`./config/install.sh` with the dependencies in `config/requirements.txt`.
