cmake_minimum_required(VERSION 3.14...3.24)
project(vtol_controller_tests LANGUAGES CXX)
include(CTest)
add_executable(pso_policy_test pso_policy_test.cpp)
target_compile_features(pso_policy_test PRIVATE cxx_std_17)
target_include_directories(
  pso_policy_test
  PRIVATE ../../src/modules/pso_control)
add_test(NAME pso_policy COMMAND pso_policy_test)
add_subdirectory(../../../companion/allocator allocator)
fetchcontent_getproperties(mavlink)
add_executable(pso_protocol_test pso_protocol_test.cpp)
target_compile_features(pso_protocol_test PRIVATE cxx_std_17)
target_include_directories(pso_protocol_test PRIVATE ../../src/modules/mavlink)
target_include_directories(
  pso_protocol_test
  SYSTEM
  PRIVATE ${mavlink_SOURCE_DIR}/common)
target_link_libraries(pso_protocol_test PRIVATE companion_core util)
add_test(
  NAME pso_protocol
  COMMAND
    pso_protocol_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../companion/allocator/src/pso/config.json)
