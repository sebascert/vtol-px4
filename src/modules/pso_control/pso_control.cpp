// SPDX-License-Identifier: BSD-3-Clause
#include "policy.hpp"
#include "vehicle_actuators.hpp"
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/pso_actuator_outputs.h>
#include <uORB/topics/pso_allocation.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

class PsoControl final : public ModuleBase<PsoControl>,
                         public px4::ScheduledWorkItem {
  uORB::Subscription allocations{ORB_ID(pso_allocation)},
      armed_sub{ORB_ID(actuator_armed)}, status_sub{ORB_ID(vehicle_status)},
      target_sub{ORB_ID(vehicle_local_position_setpoint)},
      command_sub{ORB_ID(trajectory_setpoint)},
      attitude_target_sub{ORB_ID(vehicle_attitude_setpoint)},
      position_sub{ORB_ID(vehicle_local_position)},
      attitude_sub{ORB_ID(vehicle_attitude)};
  uORB::Publication<actuator_motors_s> motors_pub{ORB_ID(actuator_motors)};
  uORB::Publication<actuator_servos_s> servos_pub{ORB_ID(actuator_servos)};
  uORB::Publication<pso_actuator_outputs_s> outputs_pub{
      ORB_ID(pso_actuator_outputs)};
  uORB::Publication<vehicle_thrust_setpoint_s> thrust_pub{
      ORB_ID(vehicle_thrust_setpoint)};
  pso_allocation_s allocation{};
  pso_allocation_s applied{};
  actuator_armed_s armed{};
  vehicle_status_s status{};
  vehicle_local_position_setpoint_s target{};
  trajectory_setpoint_s reference{};
  vehicle_attitude_setpoint_s attitude_target{};
  vehicle_local_position_s position{};
  vehicle_attitude_s attitude{};
  uint64_t enabled_since{}, last_state{};
  bool have_allocation{};
  bool was_enabled{};
  bool hil_mode{};
  bool has_applied{}, fault_latched{};
  uint8_t reason{1};

  void publish(uint64_t now, bool active) {
    pso_actuator_outputs_s output{};
    output.timestamp = now;
    output.timestamp_sample = active ? applied.timestamp_remote : 0;
    output.active = active;
    output.reason = active ? 0 : reason;
    actuator_motors_s motors{};
    actuator_servos_s servos{};
    motors.timestamp = servos.timestamp = now;
    motors.timestamp_sample = servos.timestamp_sample = output.timestamp_sample;

    for (auto &x : motors.control) {
      x = NAN;
    }

    for (auto &x : servos.control) {
      x = NAN;
    }

    for (unsigned i = 0; i < 9; ++i) {
      const float value =
          active ? applied.actuators[i] : vtol_vehicle::disarmed[i];
      output.controls[i] = vtol_pso::normalize(value, i, vtol_vehicle::minimum,
                                               vtol_vehicle::maximum);

      if (i < 3) {
        motors.control[i] = output.controls[i];
      }

      else {
        servos.control[i - 3] = output.controls[i];
      }
    }

    motors_pub.publish(motors);
    servos_pub.publish(servos);
    vehicle_thrust_setpoint_s thrust{};
    thrust.timestamp = now;
    thrust.timestamp_sample = output.timestamp_sample;

    if (active) {
      for (unsigned i = 0; i < 3; ++i) {
        const float value = applied.actuators[i] *
                            vtol_vehicle::thrust_coefficients[i] /
                            vtol_vehicle::maximum_thrust;
        const float tilt = i < 2 ? applied.actuators[3 + i] : 0.f;
        thrust.xyz[0] += value * std::sin(tilt);
        thrust.xyz[2] -= value * std::cos(tilt);
      }
    }

    thrust_pub.publish(thrust);
    outputs_pub.publish(output);
  }

  void Run() override {
    if (should_exit()) {
      reason = 1;
      publish(hrt_absolute_time(), false);
      ScheduleClear();
      exit_and_cleanup();
      return;
    }

    armed_sub.update(&armed);
    status_sub.update(&status);
    target_sub.update(&target);
    command_sub.update(&reference);
    attitude_target_sub.update(&attitude_target);
    position_sub.update(&position);
    attitude_sub.update(&attitude);
    pso_allocation_s incoming{};
    const bool allocation_updated = allocations.update(&incoming);
    // A publisher can run during these copies. Check ages against the time
    // after sampling so a newly received timestamp cannot appear in the future.
    const uint64_t now = hrt_absolute_time();
    const matrix::Quatf q{attitude_target.q_d};
    const matrix::Eulerf rpy{q};
    const float orientation[3] = {rpy(0), rpy(1), rpy(2)};
    const float desired_position[3] = {target.x, target.y, target.z};
    bool references = vtol_pso::fresh(target.timestamp, now) &&
                      vtol_pso::fresh(reference.timestamp, now) &&
                      vtol_pso::fresh(attitude_target.timestamp, now) &&
                      std::isfinite(q.norm()) &&
                      std::abs(q.norm() - 1.f) < 0.01f;

    for (unsigned i = 0; i < 3; ++i) {
      references &=
          std::isfinite(desired_position[i]) && std::isfinite(orientation[i]);
    }

    const bool enabled =
        armed.armed && !armed.lockdown && !armed.kill && !armed.termination &&
        !armed.in_esc_calibration_mode &&
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD &&
        status.hil_state == (hil_mode ? vehicle_status_s::HIL_STATE_ON
                                      : vehicle_status_s::HIL_STATE_OFF);

    if (enabled && !was_enabled) {
      enabled_since = now;
      last_state = 0;
      have_allocation = false;
    }

    was_enabled = enabled;

    if (!enabled) {
      if (armed.armed && has_applied) {
        fault_latched = true;
      }

      if (!armed.armed) {
        fault_latched = false;
      }

      enabled_since = now;
      last_state = 0;
      have_allocation = false;
      has_applied = false;
      reason = 1;
    }

    if (allocation_updated && enabled) {
      if (incoming.protocol_version == 1 &&
          vtol_pso::fresh(incoming.timestamp_remote, now) &&
          incoming.timestamp_remote > last_state) {
        allocation = incoming;
        have_allocation = true;
        last_state = incoming.timestamp_remote;
      }
    }

    const bool state_ok = references && position.xy_valid && position.z_valid &&
                          position.v_xy_valid && position.v_z_valid &&
                          vtol_pso::fresh(position.timestamp, now) &&
                          vtol_pso::fresh(attitude.timestamp, now);

    if (enabled) {
      reason = !state_ok ? 2 : 3;
    }

    const bool ready =
        enabled && state_ok && have_allocation &&
        vtol_pso::usable(allocation, now, enabled_since, desired_position,
                         orientation, vtol_vehicle::minimum,
                         vtol_vehicle::maximum);
    // A new goal may hold the accepted command only within its original age
    // limit.
    const bool valid_data =
        have_allocation &&
        vtol_pso::usable(allocation, now, enabled_since,
                         allocation.target_position,
                         allocation.target_orientation, vtol_vehicle::minimum,
                         vtol_vehicle::maximum);

    if (ready && !fault_latched) {
      applied = allocation;
      has_applied = true;
    }

    if (enabled && has_applied &&
        (!state_ok || !valid_data ||
         !vtol_pso::fresh(applied.timestamp_remote, now))) {
      fault_latched = true;
    }

    const bool active = enabled && state_ok && has_applied && !fault_latched;

    if (active) {
      reason = 0;
    }

    if (fault_latched) {
      reason = 4;
    }

    publish(now, active);
  }

public:
  PsoControl()
      : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}
  bool init() {
    int32_t autostart = 0;
    param_get(param_find("SYS_AUTOSTART"), &autostart);
    int32_t hitl = 0;
    param_get(param_find("SYS_HITL"), &hitl);
    hil_mode = vtol_pso::hil_profile(autostart, hitl);
    bool allowed = hil_mode;
#if defined(__PX4_POSIX)
    allowed |= autostart == 4501 && hitl == 0;
#endif

    if (!allowed) {
      PX4_ERR("Use SITL 4501 or external HIL 22001");
      return false;
    }

    enabled_since = hrt_absolute_time();
    PX4_INFO("vehicle config %s", vtol_vehicle::sha256);
    ScheduleOnInterval(10000);
    return true;
  }
  static int task_spawn(int, char **) {
    auto *instance = new PsoControl();

    if (!instance) {
      return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
  }
  int print_status() override {
    PX4_INFO("running, active=%d, last source time=%llu", reason == 0,
             (unsigned long long)last_state);
    return 0;
  }
  static int custom_command(int, char **) {
    return print_usage("unknown command");
  }
  static int print_usage(const char *why = nullptr) {
    if (why) {
      PX4_WARN("%s", why);
    }

    PRINT_MODULE_USAGE_NAME("pso_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
  }
};
extern "C" __EXPORT int pso_control_main(int argc, char **argv) {
  return PsoControl::main(argc, argv);
}
