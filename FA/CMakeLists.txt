cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
set(CMAKE_PROJECT_NAME MC02)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Migrate the previous cache switches once. Subsequent selection uses SCENE alone.
if(DEFINED FRAME_CDC_MOTOR_BENCH OR DEFINED FRAME_CDC_MOTOR_TELEMETRY)
    set(legacy_scene imu)
    if(FRAME_CDC_MOTOR_BENCH)
        if(NOT DEFINED FRAME_CDC_MOTOR_TELEMETRY OR FRAME_CDC_MOTOR_TELEMETRY)
            set(legacy_scene motor_bench_motor)
        else()
            set(legacy_scene motor_bench_imu)
        endif()
    endif()
    if(NOT DEFINED FRAME_CDC_SCENE)
        set(FRAME_CDC_SCENE "${legacy_scene}" CACHE STRING "YAML scene")
    endif()
    unset(FRAME_CDC_MOTOR_BENCH CACHE)
    unset(FRAME_CDC_MOTOR_TELEMETRY CACHE)
    message(STATUS "Retired legacy feature switches; FRAME_CDC_SCENE=${FRAME_CDC_SCENE}")
endif()
set(FRAME_CDC_SCENE imu CACHE STRING "Scene in the selected YAML")
set_property(CACHE FRAME_CDC_SCENE PROPERTY STRINGS imu motor_bench_imu motor_bench_motor)
set(FRAME_CDC_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/01_assembly/config/frame.yaml"
    CACHE FILEPATH "Authoritative YAML parameters and scene definitions")
get_filename_component(FRAME_CDC_CONFIG "${FRAME_CDC_CONFIG}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(config_generator "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_config.py")
set(generated_config "${CMAKE_CURRENT_BINARY_DIR}/generated/frame_config.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FRAME_CDC_CONFIG}" "${config_generator}")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${config_generator}"
        --config "${FRAME_CDC_CONFIG}" --scene "${FRAME_CDC_SCENE}"
    RESULT_VARIABLE config_result OUTPUT_VARIABLE config_features
    ERROR_VARIABLE config_error OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT config_result EQUAL 0)
    file(REMOVE "${generated_config}")
    message(FATAL_ERROR "YAML validation failed\n${config_error}")
endif()
list(GET config_features 0 frame_application)
list(GET config_features 1 frame_telemetry)
message(STATUS "Scene ${FRAME_CDC_SCENE}: ${frame_application}, ${frame_telemetry} telemetry")
add_custom_command(
    OUTPUT "${generated_config}"
    COMMAND "${Python3_EXECUTABLE}" "${config_generator}"
        --config "${FRAME_CDC_CONFIG}" --scene "${FRAME_CDC_SCENE}" --output "${generated_config}"
    DEPENDS "${FRAME_CDC_CONFIG}" "${config_generator}" "${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt"
    VERBATIM COMMENT "Validating YAML and generating const C configuration"
)

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)
include(cmake/frame_sources.cmake)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FRAME_COMMON_SOURCES}
    04_base/bsp/mc02/mc02_board.c
    04_base/devices/bmi088/bmi088.c
    "${generated_config}"
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${FRAME_INCLUDE_DIRS})
if(frame_application STREQUAL "motor_bench")
    target_compile_definitions(stm32cubemx INTERFACE HAL_FDCAN_MODULE_ENABLED HSE_VALUE=24000000UL)
    target_sources(STM32_Drivers PRIVATE Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FRAME_CDC_HAS_MOTOR_BENCH)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FRAME_MOTOR_SOURCES} 04_base/bsp/mc02/mc02_can.c)
endif()
if(frame_telemetry STREQUAL "motor")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FRAME_CDC_HAS_MOTOR_TELEMETRY)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE 02_components/telemetry/motor_telemetry.c)
else()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE 02_components/telemetry/imu_telemetry.c)
endif()
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
