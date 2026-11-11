cmake_minimum_required(VERSION 3.22)
project(frame_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(FRAME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
include("${FRAME_ROOT}/cmake/frame_sources.cmake")
list(TRANSFORM FRAME_INCLUDE_DIRS PREPEND "${FRAME_ROOT}/")
list(TRANSFORM FRAME_COMMON_SOURCES PREPEND "${FRAME_ROOT}/")
list(TRANSFORM FRAME_MOTOR_SOURCES PREPEND "${FRAME_ROOT}/")
add_compile_options(-Wall -Wextra -Werror -pedantic -UNDEBUG)
if(NOT WIN32)
    link_libraries(m)
endif()
add_executable(speed_trajectory_test speed_trajectory_test.c
    "${FRAME_ROOT}/03_algorithm/trajectory/speed_trajectory.c")
target_include_directories(speed_trajectory_test PRIVATE
    "${FRAME_ROOT}/03_algorithm/trajectory" "${FRAME_ROOT}/03_algorithm/common")
add_test(NAME speed_trajectory COMMAND speed_trajectory_test)

add_executable(speed_controller_test speed_controller_test.c
    "${FRAME_ROOT}/02_components/controller/speed_controller.c"
    "${FRAME_ROOT}/03_algorithm/trajectory/speed_trajectory.c"
    "${FRAME_ROOT}/03_algorithm/attitude/attitude.c")
target_include_directories(speed_controller_test PRIVATE
    "${FRAME_ROOT}/02_components/controller" "${FRAME_ROOT}/02_components/operator"
    "${FRAME_ROOT}/02_components/hardware/imu" "${FRAME_ROOT}/02_components/hardware/ins"
    "${FRAME_ROOT}/03_algorithm/trajectory" "${FRAME_ROOT}/03_algorithm/common"
    "${FRAME_ROOT}/03_algorithm/attitude")
add_test(NAME speed_controller COMMAND speed_controller_test)

add_executable(damiao_device_test damiao_device_test.c
    "${FRAME_ROOT}/02_components/hardware/devices/damiao/damiao_mit.c"
    "${FRAME_ROOT}/02_components/hardware/devices/damiao/damiao_motor.c"
    "${FRAME_ROOT}/04_base/services/can/can_service.c")
target_include_directories(damiao_device_test PRIVATE
    "${FRAME_ROOT}/02_components/hardware/devices/damiao"
    "${FRAME_ROOT}/04_base/bsp" "${FRAME_ROOT}/04_base/bsp/mc02"
    "${FRAME_ROOT}/04_base/services/can")
add_test(NAME damiao_device COMMAND damiao_device_test)

add_executable(motor_bench_test motor_bench_test.c ${FRAME_MOTOR_SOURCES}
    "${FRAME_ROOT}/03_algorithm/attitude/attitude.c"
    "${FRAME_ROOT}/02_components/telemetry/motor_telemetry.c"
    "${FRAME_ROOT}/02_components/telemetry/telemetry_encoding.c")
target_include_directories(motor_bench_test PRIVATE ${FRAME_INCLUDE_DIRS})
foreach(case feedback fixed pitch submission two)
    add_test(NAME motor_${case} COMMAND motor_bench_test ${case})
endforeach()

add_executable(ins_test ins_test.c
    "${FRAME_ROOT}/02_components/hardware/ins/ins.c"
    "${FRAME_ROOT}/03_algorithm/attitude/attitude.c")
target_include_directories(ins_test PRIVATE
    "${FRAME_ROOT}/02_components/hardware/imu" "${FRAME_ROOT}/02_components/hardware/ins"
    "${FRAME_ROOT}/03_algorithm/common" "${FRAME_ROOT}/03_algorithm/attitude")
add_test(NAME ins COMMAND ins_test)

set(generated_config "${CMAKE_CURRENT_BINARY_DIR}/generated/frame_config.c")
add_custom_command(OUTPUT "${generated_config}"
    COMMAND "${Python3_EXECUTABLE}" "${FRAME_ROOT}/tools/generate_config.py"
        --config "${FRAME_ROOT}/01_assembly/config/frame.yaml" --scene imu --output "${generated_config}"
    DEPENDS "${FRAME_ROOT}/tools/generate_config.py" "${FRAME_ROOT}/01_assembly/config/frame.yaml"
    VERBATIM)
add_executable(imu_frame_test imu_frame_test.c ${FRAME_COMMON_SOURCES} "${generated_config}"
    "${FRAME_ROOT}/02_components/telemetry/imu_telemetry.c")
target_include_directories(imu_frame_test PRIVATE ${FRAME_INCLUDE_DIRS})
add_test(NAME imu_frame COMMAND imu_frame_test "${CMAKE_CURRENT_BINARY_DIR}/imu_frame.bin")
add_test(NAME yaml_validation COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/config_test.py" "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_C_COMPILER} ${FRAME_INCLUDE_DIRS})
