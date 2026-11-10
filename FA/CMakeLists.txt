cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MC02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

option(FRAME_CDC_MOTOR_BENCH "Run the CAN1 MIT motor bench application" OFF)
option(FRAME_CDC_MOTOR_TELEMETRY "Use motor telemetry on UART7 in bench builds" ON)
if(FRAME_CDC_MOTOR_BENCH)
    target_compile_definitions(stm32cubemx INTERFACE HAL_FDCAN_MODULE_ENABLED HSE_VALUE=24000000UL)
    target_sources(STM32_Drivers PRIVATE Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FRAME_CDC_MOTOR_BENCH)
    if(FRAME_CDC_MOTOR_TELEMETRY)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FRAME_CDC_MOTOR_TELEMETRY)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE app/telemetry/motor_telemetry.c)
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        base/bsp/mc02/mc02_can.c
        base/services/can/can_service.c
        base/devices/damiao/damiao_mit.c
        base/devices/damiao/damiao_motor.c
        base/protocols/operator/operator_input.c
        app/motor_bench/motor_bench.c
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        base/bsp
        base/devices/damiao
        base/services/can
        base/protocols/operator
        app/motor_bench
    )
endif()
if(NOT FRAME_CDC_MOTOR_BENCH OR NOT FRAME_CDC_MOTOR_TELEMETRY)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE app/telemetry/imu_telemetry.c)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    base/bsp/mc02/mc02_board.c
    base/devices/bmi088/bmi088.c
    app/imu/imu_observer.c
    app/ins/ins.c
    base/protocols/telemetry/telemetry_encoding.c
    algorithm/attitude/attitude.c
    app/assembly/frame_system.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    base/bsp/mc02
    base/devices/bmi088
    app/imu
    app/ins
    app/telemetry
    algorithm/attitude
    algorithm/common
    base/protocols/telemetry
    app/assembly
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
