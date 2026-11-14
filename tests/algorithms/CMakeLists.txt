cmake_minimum_required(VERSION 3.22)
project(dm02_algorithm_tests LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
get_filename_component(DM02_PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${DM02_PROJECT_ROOT}/cmake/dm02_algorithms.cmake")
add_dm02_algorithms("${DM02_PROJECT_ROOT}")

add_executable(algorithm_tests native_main.cpp algorithm_cases.cpp)
target_link_libraries(algorithm_tests PRIVATE dm02_algorithms)
target_compile_options(algorithm_tests PRIVATE -Wall -Wextra)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(normal_cases basic matrix complex quaternion pid slope fsm queue frequency kalman ekf)
set(defect_cases quaternion_x axis_angle_sign pid_integral_limit pid_zero_dt
    frequency_default frequency_highpass zero_normalization singular_inverse
    kalman_predict_only invalid_mapping frequency_large_order slope_real_overshoot)
foreach(case_name IN LISTS normal_cases)
    add_test(NAME ${case_name} COMMAND algorithm_tests ${case_name})
    set_tests_properties(${case_name} PROPERTIES LABELS normal)
endforeach()
add_test(NAME diagnostic_probes
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/run_diagnostics.py"
        --compiler "${CMAKE_CXX_COMPILER}" --output "${CMAKE_CURRENT_BINARY_DIR}/diagnostics")
set_tests_properties(diagnostic_probes PROPERTIES LABELS known_defect)
foreach(case_name IN LISTS defect_cases)
    add_test(NAME defect_${case_name} COMMAND algorithm_tests ${case_name})
    # Keep real numerical failures visible: do not use WILL_FAIL or invert status.
    set_tests_properties(defect_${case_name} PROPERTIES LABELS known_defect)
endforeach()
