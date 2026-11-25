if(NOT EXISTS "${PX4_BINARY_DIR}/vehicle_actuators.hpp")
  message(
    FATAL_ERROR
      "Run config/install.sh, then controller/build.sh to generate actuator limits")
endif()
px4_add_module(
  MODULE
  modules__pso_control
  MAIN
  pso_control
  SRCS
  pso_control.cpp
  INCLUDES
  ${PX4_BINARY_DIR}
  DEPENDS
  px4_work_queue)
