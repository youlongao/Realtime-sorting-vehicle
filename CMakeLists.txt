cmake_minimum_required(VERSION 3.21)

project(Climbing_Robot LANGUAGES CXX)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "Climbing_Robot targets Raspberry Pi Linux only.")
endif()

option(CLIMBING_ROBOT_ENABLE_WARNINGS
       "Enable stricter compile warnings for development."
       ON)

set(CLIMBING_ROBOT_SOURCES
    "src/climbing_fsm.cpp"
    "src/downward_sensor.cpp"
    "src/front_distance_sensor.cpp"
    "src/front_segment.cpp"
    "src/imu_sensor.cpp"
    "src/limit_switch.cpp"
    "src/linear_actuator.cpp"
    "src/logger.cpp"
    "src/main.cpp"
    "src/mcp23017_downward_sensor.cpp"
    "src/mcp23017_driver.cpp"
    "src/mcp23017_limit_switch.cpp"
    "src/middle_drive_module.cpp"
    "src/middle_lift_module.cpp"
    "src/motion_coordinator.cpp"
    "src/motor_driver.cpp"
    "src/pca9685_driver.cpp"
    "src/pose_monitor.cpp"
    "src/rear_support_module.cpp"
    "src/robot_controller.cpp"
    "src/safety_manager.cpp"
    "src/front_balance_slider.cpp"
    "src/servo_driver.cpp"
    "src/step_detector.cpp"
    "src/utils.cpp")

add_executable(Climbing_Robot ${CLIMBING_ROBOT_SOURCES})

target_include_directories(Climbing_Robot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(Climbing_Robot PRIVATE cxx_std_20)
set_target_properties(Climbing_Robot PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
target_link_libraries(Climbing_Robot PRIVATE Threads::Threads)

find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)
find_library(GPIOD_LIBRARY gpiod REQUIRED)

target_include_directories(Climbing_Robot PRIVATE "${GPIOD_INCLUDE_DIR}")
target_link_libraries(Climbing_Robot PRIVATE "${GPIOD_LIBRARY}")

add_executable(DRV8833_Direct_Test "tools/drv8833_direct_test.cpp")
target_compile_features(DRV8833_Direct_Test PRIVATE cxx_std_20)
set_target_properties(DRV8833_Direct_Test PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_link_libraries(DRV8833_Direct_Test PRIVATE Threads::Threads "${GPIOD_LIBRARY}")
target_include_directories(DRV8833_Direct_Test PRIVATE "${GPIOD_INCLUDE_DIR}")

add_executable(DRV8833_PCA_Wheel_Test
    "tools/tb6612_pca_direct_test.cpp"
    "src/logger.cpp"
    "src/pca9685_driver.cpp"
    "src/utils.cpp")
target_compile_features(DRV8833_PCA_Wheel_Test PRIVATE cxx_std_20)
set_target_properties(DRV8833_PCA_Wheel_Test PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_include_directories(DRV8833_PCA_Wheel_Test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${GPIOD_INCLUDE_DIR}")
target_link_libraries(DRV8833_PCA_Wheel_Test PRIVATE Threads::Threads "${GPIOD_LIBRARY}")

add_executable(DRV8833_Lift_Test
    "tools/drv8833_lift_test.cpp"
)
target_compile_features(DRV8833_Lift_Test PRIVATE cxx_std_20)
set_target_properties(DRV8833_Lift_Test PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_include_directories(DRV8833_Lift_Test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${GPIOD_INCLUDE_DIR}")
target_link_libraries(DRV8833_Lift_Test PRIVATE Threads::Threads "${GPIOD_LIBRARY}")

add_executable(DRV8833_PCA_Single_Motor_Test
    "tools/drv8833_pca_single_motor_test.cpp"
    "src/logger.cpp"
    "src/pca9685_driver.cpp"
    "src/utils.cpp")
target_compile_features(DRV8833_PCA_Single_Motor_Test PRIVATE cxx_std_20)
set_target_properties(DRV8833_PCA_Single_Motor_Test PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_include_directories(DRV8833_PCA_Single_Motor_Test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${GPIOD_INCLUDE_DIR}")
target_link_libraries(DRV8833_PCA_Single_Motor_Test PRIVATE Threads::Threads "${GPIOD_LIBRARY}")

if(CLIMBING_ROBOT_ENABLE_WARNINGS)
  target_compile_options(Climbing_Robot PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(DRV8833_Direct_Test PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(DRV8833_PCA_Wheel_Test PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(DRV8833_Lift_Test PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(DRV8833_PCA_Single_Motor_Test PRIVATE -Wall -Wextra -Wpedantic)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${CLIMBING_ROBOT_SOURCES})
