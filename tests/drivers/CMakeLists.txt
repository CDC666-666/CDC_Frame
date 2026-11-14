cmake_minimum_required(VERSION 3.22)
project(dm02_driver_tests LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
file(GLOB_RECURSE DRIVER_SOURCES "${ROOT}/User_File/1_Middleware/Driver/*.cpp")
add_executable(driver_tests driver_tests.cpp ${DRIVER_SOURCES}
    "${ROOT}/User_File/2_Device/BSP/W25Q64JV/bsp_w25q64jv.cpp"
    "${ROOT}/User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp"
    "${ROOT}/User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp"
    "${ROOT}/User_File/2_Device/Motor/Motor_DJI/dvc_motor_dji.cpp"
    "${ROOT}/User_File/2_Device/Plotter/Serialplot/dvc_serialplot.cpp")
include("${ROOT}/cmake/dm02_algorithms.cmake")
add_dm02_algorithms("${ROOT}")
target_link_libraries(driver_tests PRIVATE dm02_algorithms)
target_include_directories(driver_tests PRIVATE fakes "${ROOT}/User_File")
target_compile_options(driver_tests PRIVATE -Wall -Wextra -Werror)
# 参考DM共用模式枚举，两个类各自只处理所属枚举；保留警告及原协议实现。
set_source_files_properties(
    "${ROOT}/User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp"
    PROPERTIES COMPILE_OPTIONS "-Wno-error=switch")
enable_testing()
foreach(case_name can uart framing spi adc_watchdog flash plotters usb_routing motors)
  add_test(NAME driver_${case_name} COMMAND driver_tests ${case_name})
endforeach()
