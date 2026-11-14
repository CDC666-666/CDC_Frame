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
set(CMAKE_PROJECT_NAME dm02_test)

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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

# Add LibXR
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)

# Imported pure algorithms: available to applications without starting any tasks.
include(${CMAKE_CURRENT_LIST_DIR}/cmake/dm02_algorithms.cmake)
add_dm02_algorithms(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${CMAKE_PROJECT_NAME} dm02_algorithms)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/dm02_devices.cmake)

# Link-only validation image. This target is never flashed or built by default.
add_executable(dm02_algorithms_link EXCLUDE_FROM_ALL
    tests/algorithms/arm_link_main.cpp
    tests/algorithms/algorithm_cases.cpp
    Core/Startup/startup_stm32h723vgtx.s
    Core/Src/system_stm32h7xx.c
)
target_link_libraries(dm02_algorithms_link PRIVATE dm02_algorithms stm32cubemx)
target_compile_features(dm02_algorithms_link PRIVATE cxx_std_20)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/validation")
set_target_properties(dm02_algorithms_link PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/validation")
target_link_options(dm02_algorithms_link PRIVATE --specs=nosys.specs
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/validation/dm02_algorithms_link.map")

# Prints the section sizes
function(print_section_sizes TARGET)
  # Build Events command
  add_custom_command(
    TARGET ${TARGET}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${TARGET}.elf)
endfunction()

print_section_sizes(${PROJECT_NAME})
