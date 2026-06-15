cmake_minimum_required(VERSION 3.20)

project(lattice_fusion_navstack
  VERSION 0.1.0
  DESCRIPTION "C++ autonomy navigation stack with EKF fusion, grid planning, local trajectory rollout, and lidar simulation"
  LANGUAGES CXX)

option(LFN_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(lfn_nav
  src/geometry.cpp
  src/grid_map.cpp
  src/ekf.cpp
  src/a_star.cpp
  src/lattice_planner.cpp
  src/controller.cpp
  src/lidar.cpp
  src/mission.cpp)

target_include_directories(lfn_nav PUBLIC include)
target_compile_features(lfn_nav PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(lfn_nav PRIVATE /W4 /permissive-)
else()
  target_compile_options(lfn_nav PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(lfn_nav_sim apps/nav_sim.cpp)
target_link_libraries(lfn_nav_sim PRIVATE lfn_nav)

if(LFN_BUILD_TESTS)
  enable_testing()
  add_executable(lfn_tests tests/test_navstack.cpp)
  target_link_libraries(lfn_tests PRIVATE lfn_nav)
  add_test(NAME lfn_tests COMMAND lfn_tests)
endif()
