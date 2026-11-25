# Model candidate; version 0 is diagnostic, version 1 can be gated for actuation.
uint64 timestamp             # PX4 receipt time, microseconds since boot
uint64 timestamp_remote      # v0: companion clock; v1: echoed PX4 state time in microseconds
float32[9] actuators          # Model units/order from companion/allocator/src/pso/pso_state.hpp
float32 fitness
float32 position_error_m
uint32 objective_evaluations
uint8 source_system
uint8 source_component
bool position_converged       # Solver position criterion, NOT flight readiness
uint8 protocol_version
float32[3] target_position
float32[3] target_orientation # RPY radians in NED/FRD
