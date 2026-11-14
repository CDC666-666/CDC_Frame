// SPDX-License-Identifier: Apache-2.0
// 本机数值测试与 ARM 链接验证共用的测试入口，不属于运行固件。
#ifndef FRAME_CDC_TESTS_ALGORITHMS_ALGORITHM_CASES_H_
#define FRAME_CDC_TESTS_ALGORITHMS_ALGORITHM_CASES_H_

namespace frame_cdc::algorithm_tests {

// 运行指定测试组并打印每项数值比较。
// 输入组名；返回不满足数学预期的检查数量，未知组返回 2。
int RunCase(const char* name);

}  // namespace frame_cdc::algorithm_tests

#endif  // FRAME_CDC_TESTS_ALGORITHMS_ALGORITHM_CASES_H_
