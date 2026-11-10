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
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE Components/protocols/telemetry/motor_telemetry.c)
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        BSP/mc02/mc02_can.c
        Components/services/can/can_service.c
        Components/devices/damiao/damiao_mit.c
        Components/devices/damiao/damiao_motor.c
        Components/protocols/operator/operator_input.c
        app/single_motor_control/single_motor_control.c
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        BSP
        Components/devices/damiao
        Components/services/can
        Components/protocols/operator
        app/single_motor_control
    )
endif()
if(NOT FRAME_CDC_MOTOR_BENCH OR NOT FRAME_CDC_MOTOR_TELEMETRY)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Components/protocols/telemetry/imu_telemetry.c)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    BSP/mc02/mc02_board.c
    Components/devices/bmi088/bmi088.c
    Components/devices/imu/imu_device.c
    Components/algorithms/attitude/attitude.c
    system/frame_system.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    BSP/mc02
    Components/devices/bmi088
    Components/devices/imu
    Components/algorithms/attitude
    Components/algorithms/common
    Components/protocols/telemetry
    system
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
