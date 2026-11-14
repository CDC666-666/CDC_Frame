// SPDX-License-Identifier: Apache-2.0
#include "algorithm_cases.h"

// 强制 ARM 链接器检查全部算法和实际调用的模板实现。
// 无外部输入；返回数值检查失败数。本镜像只链接，不下载或执行。
int main() {
  const char* groups[] = {"basic",     "matrix", "complex", "quaternion",
                          "pid",       "slope",  "fsm",     "queue",
                          "frequency", "kalman", "ekf"};
  int failures = 0;
  for (const char* group : groups) {
    failures += frame_cdc::algorithm_tests::RunCase(group);
  }
  return failures;
}
