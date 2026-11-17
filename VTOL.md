# Pixhawk 6C / PX4 firmware

The project firmware source is [`controller/`](./), backed by the
[sebascert/vtol-px4 fork](https://github.com/sebascert/vtol-px4). The parent
repository declares it in [`.gitmodules`](../.gitmodules); after registration,
the parent's gitlink pins the firmware commit. See **Finish the local
conversion** below for the remaining user-run Git operations.

This firmware adds diagnostic PSO allocation reception and includes the MAVLink
HIL output module for **Pixhawk 6C**, target `px4_fmu-v6c_default`. The baseline
is PX4 **v1.17.0**, commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. Hardware
flashing, on-device reception and a complete HIL loop remain untested.

## Edit the firmware directly

| Source inside controller                                                               | Project change                                                      |
| -------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| [`boards/px4/fmu-v6c/default.px4board`](boards/px4/fmu-v6c/default.px4board)           | Includes `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`                  |
| [`msg/CMakeLists.txt`](msg/CMakeLists.txt)                                             | Registers the allocation topic                                      |
| [`msg/PsoAllocation.msg`](msg/PsoAllocation.msg)                                       | Defines the diagnostic `pso_allocation` topic                       |
| [`src/modules/mavlink/mavlink_receiver.h`](src/modules/mavlink/mavlink_receiver.h)     | Adds the topic publisher                                            |
| [`src/modules/mavlink/mavlink_receiver.cpp`](src/modules/mavlink/mavlink_receiver.cpp) | Validates and publishes incoming allocations                        |
| [`src/modules/mavlink/pso_allocation.hpp`](src/modules/mavlink/pso_allocation.hpp)     | Validates the companion packet                                      |
| [`boards/px4/sitl/vtol.px4board`](boards/px4/sitl/vtol.px4board)                       | Desktop SIH communication profile; excludes optional Gazebo modules |

Make firmware changes in that repository and commit them there. The separate
patch, copied overlays and `prepare.py` workflow have been removed. The
[`communication tests`](vtol_tests/communication/) exercise the real companion
sender against the firmware helper. The Raspberry Pi image remains in
`companion/image/`.

## Finish the local conversion

Automatic approval review rejected agent staging and commits, so the checkout
currently holds the firmware changes and desktop profile **uncommitted**. Run
from this project's root to commit the firmware, publish it on the authorized
fork, and register that commit as the submodule:

```sh
git -C controller add -- \
  boards/px4/fmu-v6c/default.px4board boards/px4/sitl/vtol.px4board \
  msg/CMakeLists.txt msg/PsoAllocation.msg \
  src/modules/mavlink/mavlink_receiver.cpp src/modules/mavlink/mavlink_receiver.h \
  src/modules/mavlink/pso_allocation.hpp
git -C controller commit -m "Receive PSO allocation diagnostics and enable FMUv6C HIL outputs"
git -C controller push origin HEAD:vtol-allocator
git submodule add https://github.com/sebascert/vtol-px4.git controller
```

The final command reuses the existing checkout and stages `.gitmodules` and the
firmware gitlink. Review/stage the parent documentation, test updates and
deleted patch-workflow files separately, then commit the parent repository
yourself. A later firmware commit also needs its updated gitlink recorded in the
parent. The customized firmware commit must be pushed before another clone can
fetch it.

## Reproduce the build on Ubuntu

Once the conversion is recorded and the firmware commit published, initialize
the recorded submodule revision from this repository's root:

```sh
git submodule update --init --recursive controller
```

The ARM compiler, libraries and build utilities are installed system-wide on
this laptop. To install the same package set on another Ubuntu machine:

```sh
sudo apt-get install --no-install-recommends \
  gcc-arm-none-eabi \
  binutils-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib \
  genromfs \
  gperf
```

Other host dependencies:

- CMake
- Ninja
- Native C++ compiler
- Python 3.12 with `venv`
- Bison
- Flex

See PX4's
[Ubuntu development setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)
for host installation. Building firmware alone does not require another Gazebo
installation. Build the firmware with:

```sh
./controller/build.sh firmware
```

The script creates `controller/build/venv`, installs the versions pinned in
`controller/requirements.txt`, configures CMake and builds the requested target.
Dependencies are listed one per line and were resolved from PX4's
`Tools/setup/requirements.txt` using Python 3.12.

First builds need network access for dependencies such as Micro-CDR. The
resulting QGroundControl custom firmware file is:

```text
controller/build/firmware/px4_fmu-v6c_default.px4
```

The build in the new layout used ARM GCC 13.2.1, board ID **56**, and 1,926,896
bytes of flash (**98.01%** of this target's 1920 KiB region). Earlier artifacts
are preserved in `controller/build/artifacts/`. Future features may require
removing unused modules to fit. Compilation does not establish flight readiness.

Flash the artifact through QGroundControl's custom firmware selection when ready
for a bench test. Writing it to the Pixhawk's SD card does not replace flashing
its MCU. No automatic upload or parameter changes are made here.

## Desktop communication build

The Pixhawk MCU firmware cannot execute on Ubuntu. Build a separate native PX4
binary with the project profile; it uses the same MAVLink receiver and
`pso_allocation` topic:

```sh
./controller/build.sh sitl
```

The default target is `sitl`; `./controller/build.sh` is equivalent. Both
targets keep build output and their shared Python environment under
`controller/build/`.

This profile inherits `px4_sitl_default` and disables the Gazebo bridge,
messages and plugins. It avoids the default camera plugins' OpenCV dependency
for this communication bench. It retains the built-in SIH simulator and MAVLink
transport; the Pixhawk build configuration is unaffected.

Follow the
[three-terminal run commands](../README.md#test-live-allocatorpx4-communication).
PX4 starts its standard VTOL SIH airframe (`10043`). That simulator supplies
airspeed internally; the quadrotor example has no usable airspeed for this
allocator's input contract. Starting an additional `sensor_airspeed_sim`
instance is unnecessary and can introduce negative, noisy readings at rest.

PX4 listens on UDP `14580` and sends to `14540`. `socat` binds `14540` and
exposes a pseudo-terminal because the allocator currently supports serial
transport. In the PX4 console, use `listener pso_allocation` and repeat it for
another sample. The multi-sample listener stalls in this POSIX build; no
listener source change has been made. The local run was verified with advancing
topic timestamps and matching allocator JSON. Evidence is in ignored
`controller/build/live-allocator.jsonl`, `controller/build/live-px4-topic.txt`
and `controller/build/live-communication-verification.json`.

This is a live software communication check with upstream SIH physics. The
project's Gazebo plant, physical HIL bridge and actuator application remain
pending.

## Current packet contract

The [companion sender](../companion/allocator/src/mavlink_sender.cpp) uses
MAVLink 2 `DEBUG_FLOAT_ARRAY` (message ID 350):

| Field                   | Prototype contract                                           |
| ----------------------- | ------------------------------------------------------------ |
| Sender system/component | `245` / `191`                                                |
| `name[10]`              | `PSO_ALLOC`, including its terminating zero                  |
| `array_id`              | `0`, identifying this prototype layout                       |
| `time_usec`             | Pi steady-clock timestamp; not synchronized to PX4           |
| `data[0..2]`            | `OMEGA_I`, `OMEGA_D`, `OMEGA_T`: model rotor-speed variables |
| `data[3..4]`            | `EPSILON_I`, `EPSILON_D`: model tilt angles                  |
| `data[5..6]`            | `GAMMA_I`, `GAMMA_D`: model tail angles                      |
| `data[7..8]`            | `BETA_I`, `BETA_D`: model aileron angles                     |
| `data[9]`               | Nonnegative fitness                                          |
| `data[10]`              | Nonnegative predicted position error, metres                 |
| `data[11]`              | Position convergence flag, exactly `0` or `1`                |
| `data[12]`              | Positive integral objective-evaluation count, at most 2^24   |
| `data[13..57]`          | Unused; currently sent as zero                               |

All 13 used values must be finite. Other source IDs, names, array IDs, malformed
flags/counts and invalid diagnostics do not update `pso_allocation`. Ordinary
upstream debug-array reception remains available. IDs identify the expected
sender; this filter does not authenticate it.

PX4 adds its own boot-relative receipt `timestamp`, preserves
`timestamp_remote`, and publishes candidates/diagnostics to `pso_allocation`.
The topic has no actuator consumer. Its last sample remains cached after
transmission stops; check receipt timestamps for freshness. Pi and PX4
timestamps must not be subtracted to claim one-way latency without clock
synchronization.

Rotor values are not PWM microseconds; model angles use radians, but physical
calibration remains unresolved. A nonconverged result is still useful diagnostic
data. The receiver does not claim calibrated output limits or apply motor/servo
commands. The solver still targets its startup pose rather than accepting PX4's
demanded body force/torque.

Before control integration, define a dedicated, versioned message with
addressing, request/sequence correlation, agreed units/frames, calibrated output
mapping and validity deadlines. Add controller-demand input, timing validation
and explicit arm/disarm, timeout and fallback behavior. These are separate from
reception.

## Prove reception on the bench

After deliberately flashing this firmware, use the Pi's **TELEM2** connection
and the laptop's **USB** connection. Follow the
[wiring and telemetry stream guide](../docs/gen/guides/raspberry_pi_pixhawk_6c_wiring.md)
for 57600 baud, MAVLink 2 and the four telemetry streams required by the Pi.

1. Leave the vehicle disarmed with propellers removed. Open QGroundControl over
   USB and its MAVLink Console. On PX4, run `ver all` and `mavlink status`.
2. Start the companion on the Pi, for example
   `sudo systemctl start "vtol_allocator@$USER.service"`. JSON output proves
   numerical execution on the Pi; the next step proves PX4 receipt.
3. On PX4 run `listener pso_allocation -n 3`. Expect advancing PX4 timestamps,
   source `245/191`, nine candidates and matching fitness/error/evaluation
   count. No topic updates means reception is not yet proven.
4. Stop the Pi service and repeat. The topic may display a cached sample, but
   new receipt timestamps should stop. Restart and confirm updates resume.
5. `pwm_out_sim status` should recognize the command. It may be stopped during
   an ordinary hardware boot; inclusion in the firmware does not activate HIL.

## Harmonic and the future HIL interface

**Keep Gazebo Harmonic**: the existing simulation uses `gz-sim8`. PX4's
[Classic warning](https://docs.px4.io/main/en/sim_gazebo_classic/) describes
modern Gazebo approaching feature parity. It does not establish a stock Harmonic
USB HITL bridge. The
[official HITL guide](https://docs.px4.io/main/en/simulation/hitl) still
documents Classic and marks HITL as community maintained. Its example airframes
are HIL Quad X (`1001`) and HIL Standard VTOL (`1002`), not this vehicle.

This firmware keeps upstream `HIL_SENSOR`, `HIL_GPS` and `HIL_STATE_QUATERNION`
reception and `HIL_ACTUATOR_CONTROLS` output. It adds the documented
`pwm_out_sim` build option. HIL activation and airframe/output configuration are
separate bench steps, not defaults changed by this patch.

```text
Laptop: Gazebo Harmonic + future MAVLink HIL bridge
                    | USB: simulated sensors / HIL actuator outputs
                    v
             Pixhawk 6C / PX4 <--- TELEM2 ---> Pi / vtol_allocator
                                                   ^
Laptop: 192.168.76.1 <------- Ethernet/SSH ------ 192.168.76.2
```

The future bridge owns USB and forwards MAVLink to QGroundControl;
QGroundControl must not compete for the serial port. In HIL, upstream
`pwm_out_sim` publishes `actuator_outputs_sim`, and the HIL actuator stream
sends up to 16 normalized controls. Those are PX4 controller outputs, not the
nine raw PSO model values received by this prototype.

The [Gazebo plugin](../simulation/README.md) still applies its own ideal
forces/moments. A Harmonic-to-MAVLink sensor/actuator bridge, an actuator plant
and eventual PSO-to-PX4 output integration remain to be implemented.
`make px4_sitl gz_x500` runs software PX4, not physical Pixhawk HIL. See the
[HIL integration plan](../docs/gen/guides/gazebo_pso_hitl_plan.md).

## Host verification

```sh
cmake -S controller/vtol_tests/communication -B controller/build/host-tests -DCMAKE_BUILD_TYPE=Release &&
cmake --build controller/build/host-tests --parallel 2 &&
ctest --test-dir controller/build/host-tests --output-on-failure
```

The protocol test runs the real solver and companion sender over a
pseudo-terminal into the firmware's validation helper, including invalid-packet
cases. Full FMUv6C compilation checks the uORB/MAVLink hook. These checks do not
establish on-device reception, timing or HIL operation.

References: [PX4 overview](https://docs.px4.io/main/en/),
[development](https://docs.px4.io/main/en/development/development),
[building firmware](https://docs.px4.io/main/en/dev_setup/building_px4),
[Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/),
[pinned source](https://github.com/PX4/PX4-Autopilot/tree/d6f12ad1c4f70ad3230afd7d86e971421e02fef4).
