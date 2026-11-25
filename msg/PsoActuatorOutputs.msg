# Applied normalized controls, including explicit safe outputs when inactive.
uint64 timestamp
uint64 timestamp_sample # echoed PX4 state timestamp of applied allocation
float32[9] controls # motors [0,1], then servos [-1,1]; configured order
bool active
uint8 reason # 0 active, 1 disarmed/mode/kill, 2 stale state/target, 3 invalid/stale allocation, 4 latched fault
