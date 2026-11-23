cmake_minimum_required(VERSION 3.16)
project(uwb_imu_sandbox_gazebo LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
# Gazebo Harmonic (ROS 2 Jazzy pairing). The *_vendor packages provide them
# on a binary ROS install; a plain Gazebo install works as well.
find_package(gz_sim_vendor QUIET)
find_package(gz_transport_vendor QUIET)
find_package(gz_msgs_vendor QUIET)
find_package(gz_math_vendor QUIET)
find_package(gz_plugin_vendor QUIET)
find_package(gz_common_vendor QUIET)
find_package(sdformat_vendor QUIET)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(sdformat14 REQUIRED)

# ---------------- core library (no Gazebo dependency) ----------------
add_library(uwb_imu_sandbox_core STATIC
  src/core/sealed_model.cpp
  src/core/geometry.cpp
  src/core/uwb_model.cpp
  src/core/imu_model.cpp)
target_include_directories(uwb_imu_sandbox_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(uwb_imu_sandbox_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------- Gazebo system plugins ----------------
foreach(plugin UwbDevice RealisticImu)
  add_library(${plugin} SHARED src/plugins/${plugin}.cpp)
  target_link_libraries(${plugin} PRIVATE
    uwb_imu_sandbox_core
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-math7::gz-math7
    gz-common5::gz-common5
    sdformat14::sdformat14)
  # only link what the plugin really uses (the gz-sim8 target drags in the
  # GUI/rendering/audio stack otherwise)
  target_link_options(${plugin} PRIVATE "LINKER:--as-needed")
  install(TARGETS ${plugin} LIBRARY DESTINATION lib)
endforeach()

# ---------------- offline tools ----------------
add_executable(uwb_replay src/tools/uwb_replay.cpp)
target_link_libraries(uwb_replay uwb_imu_sandbox_core sdformat14::sdformat14 gz-math7::gz-math7)
add_executable(imu_replay src/tools/imu_replay.cpp)
target_link_libraries(imu_replay uwb_imu_sandbox_core)
install(TARGETS uwb_replay imu_replay RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core
    test/test_geometry.cpp
    test/test_sealed_model.cpp
    test/test_uwb_model.cpp
    test/test_imu_model.cpp
    TIMEOUT 300)
  target_link_libraries(test_core uwb_imu_sandbox_core)
  target_compile_definitions(test_core PRIVATE
    MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/uwb_imu_sandbox_data")
endif()

ament_package()
