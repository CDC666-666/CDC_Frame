target_include_directories(xr PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/dt7.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/dt7_protocol.cpp")
