// SPDX-License-Identifier: Apache-2.0
#include "algorithm_cases.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "1_Middleware/Algorithm/Basic/alg_basic.h"
#include "1_Middleware/Algorithm/Complex/alg_complex.h"
#include "1_Middleware/Algorithm/FSM/alg_fsm.h"
#include "1_Middleware/Algorithm/Filter/EKF/alg_filter_ekf.h"
#include "1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Algorithm/PID/alg_pid.h"
#include "1_Middleware/Algorithm/Quaternion/alg_quaternion.h"
#include "1_Middleware/Algorithm/Queue/alg_queue.h"
#include "1_Middleware/Algorithm/Slope/alg_slope.h"

namespace frame_cdc::algorithm_tests {
namespace {

// 累积数值断言，保留每项实际值和独立预期值供复核。
class Checks {
 public:
  // 比较一个数值；输入名称、实际值、预期及容差，更新失败计数。
  void Near(const char* name, double actual, double expected,
            double tolerance = 1e-5) {
    const bool passed =
        std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
    std::printf("%s %s actual=%.9g expected=%.9g tolerance=%.3g\n",
                passed ? "PASS" : "FAIL", name, actual, expected, tolerance);
    if (!passed) {
      ++failures_;
    }
  }

  // 获取累计失败数。
  // 无输入；返回当前记录器中的失败数量。
  int failures() const { return failures_; }

 private:
  int failures_ = 0;
};

// 生成一维矩阵，便于按手算模型配置标量滤波器。
// 输入数值；返回包含该数值的矩阵，保持第三方 float 接口。
Class_Matrix_f32<1, 1> Scalar(float value) {
  Class_Matrix_f32<1, 1> result;
  result[0][0] = value;
  return result;
}

// 验证基础换算、字节序、sinc 和非有限数识别。
// 输入断言记录器；记录独立数值预期的比较结果。
void Basic(Checks& checks) {
  const double pi = std::acos(-1.0);
  checks.Near("rpm_to_rad_s", BASIC_MATH_RPM_TO_RADPS * 60.0, 2.0 * pi);
  checks.Near("degrees_to_rad", BASIC_MATH_DEG_TO_RAD * 180.0, pi);
  checks.Near("celsius_to_kelvin", BASIC_MATH_CELSIUS_TO_KELVIN, 273.15);
  checks.Near("sinc_zero", Basic_Math_Sinc(0), 1);
  checks.Near("sinc_half_pi", Basic_Math_Sinc(static_cast<float>(pi / 2)),
              2.0 / pi, 3e-5);
  checks.Near("float_to_int", Basic_Math_Float_To_Int(0.5f, 0, 1, 0, 100), 50);
  checks.Near("int_to_float", Basic_Math_Int_To_Float(50, 0, 100, -1, 1), 0);
  checks.Near("modulus_negative", Basic_Math_Modulus_Normalization(-7, 10), 3);
  checks.Near("modulus_positive", Basic_Math_Modulus_Normalization(7, 10), -3);
  std::uint16_t word = 0x1234;
  Basic_Math_Endian_Reverse_16(&word);
  checks.Near("reverse16", word, 0x3412);
  std::uint32_t dword = 0x12345678;
  Basic_Math_Endian_Reverse_32(&dword);
  checks.Near("reverse32", dword, 0x78563412);
  const std::uint8_t bytes[] = {1, 2, 3};
  checks.Near("sum8", Basic_Math_Sum_8(bytes, 3), 6);
  checks.Near("sum8_empty", Basic_Math_Sum_8(bytes, 0), 0);
  checks.Near(
      "nan_invalid",
      Basic_Math_Is_Invalid_Float(std::numeric_limits<float>::quiet_NaN()), 1);
  checks.Near(
      "inf_invalid",
      Basic_Math_Is_Invalid_Float(std::numeric_limits<float>::infinity()), 1);
  checks.Near("zero_valid", Basic_Math_Is_Invalid_Float(0), 0);
}

// 验证非奇异矩阵、矩形转置和向量运算。
// 输入断言记录器；输出与手算结果的比较记录。
void Matrix(Checks& checks) {
  const float values[] = {4, 7, 2, 6};
  Class_Matrix_f32<2, 2> matrix(values);
  auto inverse = matrix.Get_Inverse();
  checks.Near("inverse00", inverse[0][0], 0.6);
  checks.Near("inverse01", inverse[0][1], -0.7);
  checks.Near("inverse10", inverse[1][0], -0.2);
  checks.Near("inverse11", inverse[1][1], 0.4);
  auto identity = matrix * inverse;
  checks.Near("product00", identity[0][0], 1);
  checks.Near("product01", identity[0][1], 0);
  checks.Near("trace", matrix.Get_Trace(), 10);
  const float rectangular_values[] = {1, 2, 3, 4, 5, 6};
  Class_Matrix_f32<2, 3> rectangular(rectangular_values);
  auto transpose = rectangular.Get_Transpose();
  checks.Near("transpose21", transpose[2][1], 6);
  checks.Near("last_column", rectangular.Get_Column(2)[1][0], 6);
  checks.Near("last_row", rectangular.Get_Row(1)[0][2], 6);
  auto x = Namespace_ALG_Matrix::Axis_X_3d();
  auto y = Namespace_ALG_Matrix::Axis_Y_3d();
  checks.Near("cross_xy_z", Namespace_ALG_Matrix::Operator_Cross_3d(x, y)[2][0],
              1);
  checks.Near("dot_xy", Namespace_ALG_Matrix::Operator_Dot(x, y), 0);
  checks.Near("unit_norm", x.Get_Modulus(), 1);
}

// 验证复数乘除、共轭和二维旋转。
// 输入断言记录器；记录代数恒等式及已知旋转结果。
void Complex(Checks& checks) {
  Class_Complex_f32 a(1, 2);
  Class_Complex_f32 b(3, 4);
  auto product = a * b;
  checks.Near("complex_product_real", product.Get_Real(), -5);
  checks.Near("complex_product_imag", product.Get_Imaginary(), 10);
  auto recovered = product / b;
  checks.Near("complex_division_real", recovered.Get_Real(), 1);
  checks.Near("complex_division_imag", recovered.Get_Imaginary(), 2);
  checks.Near("conjugate", a.Get_Conjugate().Get_Imaginary(), -2);
  checks.Near("complex_inverse", (a * a.Get_Inverse()).Get_Real(), 1);
  auto rotation = Class_Complex_f32(0, 2).Get_Rotation_Matrix();
  checks.Near("complex_rotation10", rotation[1][0], 1);
  checks.Near("complex_rotation01", rotation[0][1], -1);
}

// 验证 Hamilton 四元数乘积和绕 Z 轴正向九十度旋转。
// 输入断言记录器；预期由单位基及三角函数独立给出。
void Quaternion(Checks& checks) {
  const float half_pi = static_cast<float>(std::acos(-1.0) / 2);
  const float values[] = {half_pi, 0, 0};
  Class_Matrix_f32<3, 1> angles(values);
  auto q = Namespace_ALG_Quaternion::From_Euler_Angle(angles);
  checks.Near("quaternion_real", q.Get_Real(), std::sqrt(0.5));
  checks.Near("quaternion_z", q.Get_Imaginary()[2][0], std::sqrt(0.5));
  auto rotation = q.Get_Rotation_Matrix();
  checks.Near("quaternion_rotate_x_y", rotation[1][0], 1);
  checks.Near("quaternion_yaw", q.Get_Euler_Angle()[0][0], half_pi);
  auto product = q * q.Get_Inverse();
  checks.Near("quaternion_inverse_real", product.Get_Real(), 1);
  checks.Near("quaternion_inverse_z", product.Get_Imaginary()[2][0], 0);
  auto from_axis = Namespace_ALG_Quaternion::From_Axis_Angle(
      Namespace_ALG_Matrix::Axis_Z_3d(), half_pi);
  checks.Near("axis_quaternion_z", from_axis.Get_Imaginary()[2][0],
              std::sqrt(0.5), 3e-5);
  auto from_matrix = Namespace_ALG_Quaternion::From_Rotation_Matrix(rotation);
  checks.Near("matrix_quaternion_real", from_matrix.Get_Real(), std::sqrt(0.5));
}

// 验证 PID 的比例、积分、微分及输出限幅。
// 输入断言记录器；独立配置各项，记录逐步手算预期。
void Pid(Checks& checks) {
  Class_PID proportional;
  proportional.Init(2, 0, 0);
  proportional.Set_Target(3);
  proportional.Set_Now(1);
  proportional.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_proportional", proportional.Get_Out(), 4);
  Class_PID integral;
  integral.Init(0, 1, 0, 0, 0, 0, 0.1f);
  integral.Set_Target(2);
  integral.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_integral_step1", integral.Get_Out(), 0.2);
  integral.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_integral_step2", integral.Get_Out(), 0.4);
  Class_PID derivative;
  derivative.Init(0, 0, 1, 0, 0, 0, 0.1f);
  derivative.Set_Target(1);
  derivative.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_derivative_step", derivative.Get_Out(), 10);
  derivative.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_derivative_constant", derivative.Get_Out(), 0);
  Class_PID limited;
  limited.Init(10, 0, 0, 0, 0, 3);
  limited.Set_Target(1);
  limited.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("pid_output_limit", limited.Get_Out(), 3);
}

// 验证按调用次数限制增减量的目标优先斜坡。
// 输入断言记录器；检查上升和下降两种方向。
void Slope(Checks& checks) {
  Class_Slope slope;
  slope.Init(1, 2, Slope_First_TARGET);
  slope.Set_Target(10);
  slope.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("slope_step1", slope.Get_Out(), 1);
  slope.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("slope_step2", slope.Get_Out(), 2);
  slope.Set_Target(0);
  slope.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("slope_decrease", slope.Get_Out(), 0);
  slope.Set_Target(-2);
  slope.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("slope_negative", slope.Get_Out(), -1);
}

// 验证有限状态机的初始化、计数和有效范围内转移。
// 输入断言记录器；输出状态和计数断言。
void Fsm(Checks& checks) {
  Class_FSM<3> fsm;
  fsm.Init(1);
  checks.Near("fsm_initial", fsm.Get_Now_Status_Serial(), 1);
  checks.Near("fsm_enabled", fsm.Status[1].Status_Stage, Status_Stage_ENABLE);
  fsm.TIM_Calculate_PeriodElapsedCallback();
  checks.Near("fsm_tick", fsm.Status[1].Count_Time, 1);
  fsm.Set_Status(2);
  checks.Near("fsm_transition", fsm.Get_Now_Status_Serial(), 2);
  checks.Near("fsm_reset_count", fsm.Status[1].Count_Time, 0);
}

// 验证循环队列的满队列、环回、FIFO 和清空。
// 输入断言记录器；不读取未定义的空队列元素。
void Queue(Checks& checks) {
  Class_Queue<int, 3> queue;
  queue.Init();
  checks.Near("queue_empty", queue.Get_Length(), 0);
  queue.Push(1);
  queue.Push(2);
  queue.Push(3);
  queue.Push(99);
  checks.Near("queue_full", queue.Get_Length(), 3);
  checks.Near("queue_fifo1", queue.Pop(), 1);
  queue.Push(4);
  checks.Near("queue_wrap_rear", queue.Get_Rear(), 4);
  checks.Near("queue_fifo2", queue.Pop(), 2);
  checks.Near("queue_fifo3", queue.Pop(), 3);
  checks.Near("queue_fifo4", queue.Pop(), 4);
  checks.Near("queue_drained", queue.Get_Length(), 0);
  queue.Push(5);
  queue.Clear();
  checks.Near("queue_clear", queue.Get_Length(), 0);
}

// 验证四阶低通 FIR 的单位冲激响应及直流稳态。
// 输入断言记录器；系数由标准 sinc 公式独立计算。
void Frequency(Checks& checks) {
  Class_Filter_Frequency<4> filter;
  filter.Init(-10, 10, Filter_Frequency_Type_LOWPASS, 100, 500, 1000);
  double coefficients[5]{};
  double sum = 0;
  const double omega = 2.0 * std::acos(-1.0) * 100 / 1000;
  for (int i = 0; i < 5; ++i) {
    const double x = (i - 2) * omega;
    coefficients[i] = omega / std::acos(-1.0) * (i == 2 ? 1 : std::sin(x) / x);
    sum += coefficients[i];
  }
  for (int i = 0; i < 5; ++i) {
    filter.Set_Now(i == 0 ? 1 : 0);
    filter.TIM_Calculate_PeriodElapsedCallback();
    checks.Near("fir_impulse", filter.Get_Out(), coefficients[i] / sum, 3e-5);
  }
  for (int i = 0; i < 5; ++i) {
    filter.Set_Now(1);
    filter.TIM_Calculate_PeriodElapsedCallback();
  }
  checks.Near("fir_lowpass_dc", filter.Get_Out(), 1);
}

// 验证标量 Kalman 的一次预测、增益和 Joseph 协方差更新。
// 输入断言记录器；预期 K=1/2、x=1、P=1/2。
void Kalman(Checks& checks) {
  Class_Filter_Kalman<1, 1, 1> filter;
  filter.Init(Scalar(1), Scalar(0), Scalar(1), Scalar(0), Scalar(1));
  filter.Vector_Z = Scalar(2);
  filter.TIM_Predict_PeriodElapsedCallback();
  filter.TIM_Update_PeriodElapsedCallback();
  checks.Near("kalman_gain", filter.Matrix_K[0][0], 0.5);
  checks.Near("kalman_state", filter.Vector_X[0][0], 1);
  checks.Near("kalman_covariance", filter.Matrix_P[0][0], 0.5);
}

// 为 EKF 提供线性运动模型。
// 输入状态、输入量和步长，返回下一状态。
Class_Matrix_f32<1, 1> StateModel(const Class_Matrix_f32<1, 1>& state,
                                  const Class_Matrix_f32<1, 1>& input,
                                  const float& dt_s) {
  return Scalar(state[0][0] + input[0][0] * dt_s);
}

// 提供状态及噪声雅可比。
// 输入模型参数，返回一维单位矩阵。
Class_Matrix_f32<1, 1> StateJacobian(const Class_Matrix_f32<1, 1>&,
                                     const Class_Matrix_f32<1, 1>&,
                                     const float&) {
  return Scalar(1);
}

// 提供二次测量模型。
// 输入状态和步长，返回 x 的平方。
Class_Matrix_f32<1, 1> Measurement(const Class_Matrix_f32<1, 1>& state,
                                   const float&) {
  return Scalar(state[0][0] * state[0][0]);
}

// 提供二次测量雅可比。
// 输入状态和步长，返回 2x。
Class_Matrix_f32<1, 1> MeasurementJacobian(const Class_Matrix_f32<1, 1>& state,
                                           const float&) {
  return Scalar(2 * state[0][0]);
}

// 提供测量噪声雅可比。
// 输入状态和步长，返回一维单位矩阵。
Class_Matrix_f32<1, 1> MeasurementNoise(const Class_Matrix_f32<1, 1>&,
                                        const float&) {
  return Scalar(1);
}

// 验证具有非线性测量的 EKF 更新。
// 输入断言记录器；独立预期为 K=4/17、x=38/17、P=1/17。
void Ekf(Checks& checks) {
  Class_Filter_EKF<1, 1, 1> filter;
  filter.Init(Scalar(0), Scalar(1), Scalar(1), Scalar(2), Scalar(0));
  filter.Set_D_T(0.01f);
  filter.Config_Nonlinear_State_Model(StateModel, StateJacobian, StateJacobian);
  filter.Config_Nonlinear_Measurement_Model(Measurement, MeasurementJacobian,
                                            MeasurementNoise);
  filter.Vector_Z = Scalar(5);
  filter.TIM_Predict_PeriodElapsedCallback();
  filter.TIM_Update_PeriodElapsedCallback();
  checks.Near("ekf_gain", filter.Matrix_K[0][0], 4.0 / 17);
  checks.Near("ekf_state", filter.Vector_X[0][0], 38.0 / 17);
  checks.Near("ekf_covariance", filter.Matrix_P[0][0], 1.0 / 17);
}

// 只在测试中读取原算法的循环索引，不改变算法存储和更新逻辑。
class FrequencyIndexProbe : public Class_Filter_Frequency<256> {
 public:
  // 获取循环索引。
  // 无输入，返回当前数组写入位置。
  std::uint32_t signal_index() const { return Signal_Flag; }
};

// 运行待确认问题的数学预期；不改变原算法、不反转失败状态。
// 输入问题名和记录器；已覆盖该问题返回 true，否则返回 false。
bool Defect(const char* name, Checks& checks) {
  if (std::strcmp(name, "quaternion_x") == 0) {
    auto q = Namespace_ALG_Quaternion::Unit_Imaginary_X();
    checks.Near("unit_x_real", q.Get_Real(), 0);
    checks.Near("unit_x_imaginary_x", q.Get_Imaginary()[0][0], 1);
  } else if (std::strcmp(name, "axis_angle_sign") == 0) {
    const float angle = 1;
    auto q = Namespace_ALG_Quaternion::From_Axis_Angle(
        -Namespace_ALG_Matrix::Axis_X_3d(), angle);
    auto axis_angle = q.Get_Axis_Angle();
    auto recovered_axis = Class_Matrix_f32<3, 1>(axis_angle.Data);
    auto recovered = Namespace_ALG_Quaternion::From_Axis_Angle(
        recovered_axis, axis_angle[3][0]);
    checks.Near("axis_angle_roundtrip_yz",
                recovered.Get_Rotation_Matrix()[2][1],
                q.Get_Rotation_Matrix()[2][1], 5e-5);
  } else if (std::strcmp(name, "pid_integral_limit") == 0) {
    Class_PID pid;
    pid.Init(0, 1, 0, 0, 1, 0, 1);
    pid.Set_Target(10);
    pid.TIM_Calculate_PeriodElapsedCallback();
    checks.Near("integral_limit", pid.Get_Out(), 1);
  } else if (std::strcmp(name, "pid_zero_dt") == 0) {
    Class_PID pid;
    pid.Init(0, 0, 0, 0, 0, 0, 0);
    pid.TIM_Calculate_PeriodElapsedCallback();
    checks.Near("zero_dt_finite", std::isfinite(pid.Get_Out()), 1);
  } else if (std::strcmp(name, "frequency_default") == 0) {
    Class_Filter_Frequency<4> filter;
    filter.Init();
    filter.TIM_Calculate_PeriodElapsedCallback();
    checks.Near("default_fir_finite", std::isfinite(filter.Get_Out()), 1);
  } else if (std::strcmp(name, "frequency_highpass") == 0) {
    Class_Filter_Frequency<4> filter;
    filter.Init(-10, 10, Filter_Frequency_Type_HIGHPASS, 0, 100, 1000);
    for (int i = 0; i < 5; ++i) {
      filter.Set_Now(1);
      filter.TIM_Calculate_PeriodElapsedCallback();
    }
    checks.Near("highpass_dc_rejection", filter.Get_Out(), 0);
  } else if (std::strcmp(name, "zero_normalization") == 0) {
    auto result = Namespace_ALG_Matrix::Zero<3, 1>().Get_Normalization();
    checks.Near("zero_normalization_finite", std::isfinite(result[0][0]), 1);
  } else if (std::strcmp(name, "singular_inverse") == 0) {
    const float values[] = {1, 2, 2, 4};
    Class_Matrix_f32<2, 2> singular(values);
    auto result = singular * singular.Get_Inverse();
    checks.Near("singular_inverse_residual11", result[1][1], 1);
  } else if (std::strcmp(name, "kalman_predict_only") == 0) {
    Class_Filter_Kalman<1, 1, 1> filter;
    filter.Init(Scalar(2), Scalar(0), Scalar(1), Scalar(0), Scalar(1),
                Scalar(1), Scalar(1));
    filter.TIM_Predict_PeriodElapsedCallback();
    filter.TIM_Predict_PeriodElapsedCallback();
    checks.Near("predict_only_state", filter.Vector_X_Prior[0][0], 4);
    checks.Near("predict_only_covariance", filter.Matrix_P_Prior[0][0], 16);
  } else if (std::strcmp(name, "invalid_mapping") == 0) {
    checks.Near("zero_mapping_range_finite",
                std::isfinite(Basic_Math_Int_To_Float(0, 0, 0, 0, 1)), 1);
  } else if (std::strcmp(name, "frequency_large_order") == 0) {
    FrequencyIndexProbe filter;
    filter.Init(-10, 10, Filter_Frequency_Type_LOWPASS, 100, 500, 1000);
    for (int i = 0; i < 256; ++i) {
      filter.Set_Now(1);
    }
    checks.Near("fir_order256_index", filter.signal_index(), 256);
  } else if (std::strcmp(name, "slope_real_overshoot") == 0) {
    Class_Slope slope;
    slope.Init(1, 1, Slope_First_REAL);
    slope.Set_Now_Real(9.5f);
    slope.Set_Target(10);
    slope.TIM_Calculate_PeriodElapsedCallback();
    checks.Near("real_priority_slope_target", slope.Get_Out(), 10);
  } else {
    return false;
  }
  return true;
}

}  // namespace

// 分派算法测试；输入组名，返回实际失败数或未知组错误码。
// 所有被测对象均为局部对象，不注册到固件或 RTOS。
int RunCase(const char* name) {
  Checks checks;
  if (std::strcmp(name, "basic") == 0) {
    Basic(checks);
  } else if (std::strcmp(name, "matrix") == 0) {
    Matrix(checks);
  } else if (std::strcmp(name, "complex") == 0) {
    Complex(checks);
  } else if (std::strcmp(name, "quaternion") == 0) {
    Quaternion(checks);
  } else if (std::strcmp(name, "pid") == 0) {
    Pid(checks);
  } else if (std::strcmp(name, "slope") == 0) {
    Slope(checks);
  } else if (std::strcmp(name, "fsm") == 0) {
    Fsm(checks);
  } else if (std::strcmp(name, "queue") == 0) {
    Queue(checks);
  } else if (std::strcmp(name, "frequency") == 0) {
    Frequency(checks);
  } else if (std::strcmp(name, "kalman") == 0) {
    Kalman(checks);
  } else if (std::strcmp(name, "ekf") == 0) {
    Ekf(checks);
  } else if (!Defect(name, checks)) {
    std::printf("UNKNOWN CASE %s\n", name);
    return 2;
  }
  return checks.failures();
}

}  // namespace frame_cdc::algorithm_tests
