// SPDX-License-Identifier: Apache-2.0
#include "algorithm_cases.h"

// 在电脑端执行单个测试组。
// 输入命令行组名；检查通过返回 0，失败或参数错误返回非零。
int main(int argc, char** argv) {
  if (argc != 2) {
    return 2;
  }
  return frame_cdc::algorithm_tests::RunCase(argv[1]) == 0 ? 0 : 1;
}
