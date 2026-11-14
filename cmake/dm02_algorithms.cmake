# Build the imported algorithms against the existing CMSIS-DSP sources.
# The same targets are used by firmware and native numerical tests.
function(add_dm02_algorithms project_root)
    set(user_file_root "${project_root}/User_File")
    set(algorithm_root "${user_file_root}/1_Middleware/Algorithm")
    set(dsp_root "${project_root}/Drivers/CMSIS/DSP")

    add_library(dm02_dsp_math STATIC
        "${dsp_root}/Source/FastMathFunctions/arm_sin_f32.c"
        "${dsp_root}/Source/FastMathFunctions/arm_cos_f32.c"
        "${dsp_root}/Source/CommonTables/arm_common_tables.c"
    )
    target_include_directories(dm02_dsp_math PUBLIC
        "${dsp_root}/Include" "${project_root}/Drivers/CMSIS/Include")
    target_compile_definitions(dm02_dsp_math PRIVATE
        ARM_DSP_CONFIG_TABLES ARM_FAST_ALLOW_TABLES ARM_TABLE_SIN_F32)
    target_compile_options(dm02_dsp_math PRIVATE -ffunction-sections -fdata-sections)

    add_library(dm02_algorithms STATIC
        "${algorithm_root}/Basic/alg_basic.cpp"
        "${algorithm_root}/Complex/alg_complex.cpp"
        "${algorithm_root}/Filter/EKF/alg_filter_ekf.cpp"
        "${algorithm_root}/Filter/Frequency/alg_filter_frequency.cpp"
        "${algorithm_root}/Filter/Kalman/alg_filter_kalman.cpp"
        "${algorithm_root}/FSM/alg_fsm.cpp"
        "${algorithm_root}/Matrix/alg_matrix.cpp"
        "${algorithm_root}/PID/alg_pid.cpp"
        "${algorithm_root}/Quaternion/alg_quaternion.cpp"
        "${algorithm_root}/Queue/alg_queue.cpp"
        "${algorithm_root}/Slope/alg_slope.cpp"
    )
    target_compile_features(dm02_algorithms PUBLIC cxx_std_20)
    target_include_directories(dm02_algorithms PUBLIC "${user_file_root}")
    target_compile_options(dm02_algorithms PRIVATE
        -Wall -Wextra -ffunction-sections -fdata-sections)
    target_link_libraries(dm02_algorithms PUBLIC dm02_dsp_math)
endfunction()
