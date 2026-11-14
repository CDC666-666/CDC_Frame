# Project paths differ from upstream defaults. Reuse the pinned freshness checker.
set(frame_cdc_module_dir "${CMAKE_CURRENT_SOURCE_DIR}/User_File/3_Modules")
set(frame_cdc_user_dir "${CMAKE_CURRENT_SOURCE_DIR}/User_File/4_Task/User")
include("${frame_cdc_module_dir}/CMakeLists.txt")
include("${frame_cdc_module_dir}/local_modules.cmake")
set(frame_cdc_header "${frame_cdc_user_dir}/xrobot_main.hpp")
if(NOT EXISTS "${frame_cdc_header}")
    message(FATAL_ERROR "Run python tools/xrobot_project.py setup first")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${frame_cdc_header}")
add_custom_target(xrobot_freshness_check ALL
    COMMAND ${CMAKE_COMMAND}
        "-DXROBOT_MAIN_HEADER=${frame_cdc_header}"
        "-DXROBOT_BSP_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
        -P "${LIBXR_SOURCE_DIR}/cmake/XRobotFreshness.cmake"
    COMMENT "Checking generated XRobot configuration in User_File/4_Task/User"
    VERBATIM)
add_dependencies(${CMAKE_PROJECT_NAME} xrobot_freshness_check)
get_property(frame_cdc_module_deps GLOBAL PROPERTY XR_MODULE_DEPS)
if(frame_cdc_module_deps)
    list(REMOVE_DUPLICATES frame_cdc_module_deps)
    target_link_libraries(${CMAKE_PROJECT_NAME} ${frame_cdc_module_deps})
endif()
