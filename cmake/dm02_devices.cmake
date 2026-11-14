# Reference devices retain their source directory and class names.
file(GLOB_RECURSE DM02_DEVICE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/User_File/1_Middleware/Driver/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/User_File/1_Middleware/System/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/User_File/2_Device/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/User_File/4_Task/*.cpp")
add_library(dm02_devices STATIC ${DM02_DEVICE_SOURCES})
target_compile_features(dm02_devices PUBLIC cxx_std_20)
target_include_directories(dm02_devices PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/User_File")
target_link_libraries(dm02_devices PUBLIC dm02_algorithms xr stm32cubemx)
target_link_libraries(${CMAKE_PROJECT_NAME} dm02_devices)
