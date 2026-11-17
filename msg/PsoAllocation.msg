# Diagnostic model candidate from the Raspberry Pi; no actuator-output consumer.
uint64 timestamp             # PX4 receipt time, microseconds since boot
uint64 timestamp_remote      # Pi steady-clock time, NOT synchronized to PX4
float32[9] actuators          # Model units/order from companion/allocator/src/pso/pso_state.hpp
float32 fitness
float32 position_error_m
uint32 objective_evaluations
uint8 source_system
uint8 source_component
bool position_converged       # Solver position criterion, NOT flight readiness
