# dm02_test：MC02 基础配置与 FreeRTOS / LibXR

工程位于 `bsp-dev-mc02`，当前 CubeMX 工程及固件名称为 `dm02_test`。MCU 为 STM32H723VGT6；外设配置以指定的本地 `damiao_mc02_bsp-main/damiao_mc02_bsp-main/dm02_test` 为参考。

底层配置和生成层已按参考对齐，保留 FreeRTOS、LibXR/XRobot、70份参考核心源码和22份原字节算法。按用户最新要求，frame-cdc四个运行模块已删除；当前应用为User_File/4_Task/rm_music，调用既有蜂鸣器组件上电播放一次用户提供音频的自动转录谱。算法仍有已记录且未修复的问题；编译和主机测试不代表实板验证。

## 配置与运行入口

| 项目 | 配置 |
|---|---|
| 时钟 / Cache | 24 MHz HSE，480 MHz SYSCLK；I-Cache 开，D-Cache 关 |
| 工具链 | CMake / Ninja / Arm GNU；C11、C++20；保留 ST Arm Clang 备用工具链 |
| 固件包 / 内核 | STM32Cube FW_H7 V1.13.0 / HAL 1.11.6 / FreeRTOS V10.6.2 |
| 通信 | 参考 UART/SPI 配置，三路 Classic CAN；USB 由 LibXR 提供单 CDC 和终端 |
| 新增生成层 | OCTOSPI2、MDMA、CORDIC、IWDG1、参考 TIM4/5/6/7/8 配置 |
| 原工程扩展 | 保留 DAC1 通道2、LCD 的 PB10/PB11/PE15、SPI6 BDMA；取消 ADC19，PA5 用作 SPI6_SCK |
| 系统时基 | TIM17 为 HAL 和 LibXR 提供 1 ms 时基；SysTick 为 RTOS 调度节拍 |
| 看门狗 | 名义超时 8192 ms，独立高优先级线程每 1000 ms 喂狗，栈参数 1024 B |

启动链为 `main → defaultTask → app_main → XROBOT_MAIN`。产品YAML构造RmMusic，独占TIM12 CH2蜂鸣器；enabled: true播放一次，false静音，改后需重新生成/编译/烧录。有效谱使用2048 B低优先级线程，结束后静音并永久阻塞；禁用或缺谱时不创建播放线程。BuzzerAlarm不再实例化，避免其自带开机音绕过开关；相应自定义致命错误报警也不再安装。USB终端恢复官方自动线程，旧ref诊断/演示/采样/Flash后台不再运行。参考TIM4/7/8只初始化，PC13/PC14/PC15按参考初始化为低电平。

IWDG 的 IOC 保留参考分频/重载值；LibXR 启动时根据名义超时重新计算等效寄存器配置并关闭窗口约束。基础周期喂狗不等于业务任务健康监测，实际超时还受 LSI 频率影响。

## 文件与生成边界

- `dm02_test.ioc` 是唯一根 IOC；生成的初始化文件在 `Core/Inc`、`Core/Src`。
- 启动汇编为 `Core/Startup/startup_stm32h723vgtx.s`，链接脚本为 `STM32H723VGTX_FLASH.ld`。
- `User_File/4_Task/User/app_main.cpp`、`User_File/4_Task/User/app_main.h`、`User_File/4_Task/User/flash_map.hpp` 由固定版本生成工具生成；自定义内容仅放在 User Code 区。
- `User_File/3_Modules/modules.yaml`、`xrobot.lock` 固定应用模块，`User_File/4_Task/User/libxr_config.yaml` 固定生成器与硬件封装设置。
- LibXR 子模块固定为 `5161373dbe6d9d9fb1904d90e0ebd4344a44047a`；BuzzerAlarm及新增DurationStatistics的固定提交由 `xrobot.lock` 记录，不自动更新。

参考 GPIO 标签包含双下划线。IOC 和 HAL 宏保持参考名称；`tools/generate_libxr.py` 在固定 libxr 6.0.1 的对象命名适配点合并连续下划线，保持其余标识符/冲突检查有效。例如 `BMI088_ACCEL__SPI_CS_Pin` 对应 LibXR 对象 `BMI088_ACCEL_SPI_CS`。不要直接运行未适配的 `libxr stm32 setup` 覆盖生成物，统一使用下方入口。

SPI6 只有 TX DMA 配置，libxr 6.0.1 生成 `UINT32_MAX` 阈值选择轮询路径；BDMA 配置保留，但当前封装不会用它发起发送。没有同时引入 ST USB Device 栈。

链接脚本保留原 DTCM/TLS、DMA 缓冲段和堆栈布局。固件 Flash 限制为前 768 KiB；最后两个 128 KiB 扇区（从 `0x080C0000` 开始）留给现有 LibXR Database，禁止将程序链接到该区域。

## 本机生成与编译

先恢复固定 LibXR 子模块，并创建项目内工具环境：

```powershell
git submodule update --init --recursive
python -m venv build/tooling-env
build/tooling-env/Scripts/python.exe -m pip install -r tools/requirements.txt
build/tooling-env/Scripts/python.exe tools/regenerate.py --libxr-only
```

完整 CubeMX 生成使用已安装的 V1.13.0 固件包，输出先进入 `build/regenerate` 隔离目录，再合入生成层并保留 USER CODE 区：

```powershell
build/tooling-env/Scripts/python.exe tools/regenerate.py --cubemx D:/Apps/Stm32cubemx/STM32CubeMX.exe --firmware-root C:/Users/Lenovo/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0 --offline
build/tooling-env/Scripts/python.exe tools/check_baseline.py
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

工具路径须按安装位置调整；Arm GCC、CMake、Ninja 需可被找到。本机 VS Code 配置已指向现有工具路径。`--offline` 要求 XRobot 模块已恢复；首次恢复使用不带该选项的 `--libxr-only` 命令。生成过程保留 SDK 原件、原 FreeRTOS 用户补充及自定义链接脚本，不升级依赖。

产物为 `build/Debug/dm02_test.elf`、`build/Release/dm02_test.elf` 及对应 hex/bin/map。`DevMC02.jdebug` 仅更新固件路径，本次未运行下载或调试。

CI 使用同一 GPIO 名称适配入口，检查生成一致性及参考配置后构建 Debug/Release。只上传构建产物，不自动发布或烧录；远端 CI 结果与本机编译结果分别记录。

## 纯算法库

`User_File/1_Middleware/Algorithm` 按参考工程原目录和文件名保存11类纯算法，原接口、float计算和22份源码字节均保留。应用可直接包含 `1_Middleware/Algorithm/...`；独立静态库 `dm02_algorithms` 已接入构建，参考设备及Kalman演示使用其中算法。后续迁入的参考功能也沿用参考目录和命名。

原算法中已发现计算缺陷、未初始化读取、越界与接口限制，按用户决定原样保留。11组常规测试通过不等于全部算法正确；13组缺陷/限制诊断维持真实失败。运行步骤见 [测试说明](../../readme/框架/ALGORITHM_TESTS.md)，逐项问题和证据见 [算法迁移说明](../../readme/框架/ALGORITHM_MIGRATION_20261114.md)。User_File只保留功能代码，说明统一在集中资料目录。

## 许可与来源

现有 BSP 许可为 Apache-2.0，见 [LICENSE](LICENSE) 与 [NOTICE](NOTICE)。新增外设配置、算法及设备参考用户指定的本地 dm02_test，保留原有版权声明；原参考应用文件未发现独立许可，不能用本仓库许可替代其授权。项目规范与当前开发记录见 [集中入口](../../readme/框架/PROJECT_CONTEXT.md)。

## 参考设备与驱动

`User_File`保留唯一参考的70个核心文件名/层级：22份算法原件、48份Driver/System/设备适配；原Task两文件按用户要求删除并在来源清单中单独记录。当前仅rm_music经YAML和官方生成入口装配；USB由官方终端线程读取。旧ref诊断、调参分流、IMU/加热、绘图、电机示例和Flash后台已撤下，相关设备/驱动作为库保留，后续按需求重新装配。

功能清单、调用边界、命令及待实板验收项见 [使用说明](../../readme/框架/USER_FILE_README.md)。适配快照由`tools/check_reference_devices.py`校验；主机协议测试位于`tests/drivers`，HAL/LibXR替身通过不代表真实DMA、时序或外设通过。当前实测与限制只以集中状态为准。

`User_Config/daplink.cfg`、`stlink.cfg`沿用参考原件；仅做过OpenOCD禁用初始化的配置解析，未连接探针。ST-Link旧接口脚本有弃用警告，DAPLink的FLASH_SIZE变量在当前目标脚本中未使用；不能据配置存在声称烧录、Flash容量或保护区域已验证。

## 自定义目录及模块开发

2026-11-15按用户要求，根Modules/User已移到User_File/3_Modules和User_File/4_Task/User。RmMusic任务使用V2 manifest、构造参数及YAML实例；默认/静音产品见对应xrobot.yaml、minimal.yaml。播放开关和线程栈容量由YAML构造参数指定。

固定上游工具不原生支持此目录，使用项目入口：`build/tooling-env/Scripts/python.exe tools/xrobot_project.py setup --frozen`、`gen`、`describe`及`instance ...`；完整生成仍用tools/regenerate.py。安装包和LibXR原件未改，CMake继续使用官方输入摘要检查。自有模块随BSP管理，外部模块仍由真实lock固定；原生VS Code模块树目录发现未适配，已提供项目生成/恢复任务。完整说明见[模块组织](../../readme/框架/MODULE_ORGANIZATION.md)。后续开发必须先用本地xrobot skill，并核对对应官方文档/开源源码。
