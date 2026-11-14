# Frame_CDC BSP 编码入口

- 先读 [项目入口](../AGENTS.md)、[集中规则](../../readme/框架/AGENTS.md) 和 [唯一状态](../../readme/框架/PROJECT_CONTEXT.md)，核对 Git 根、分支及已有修改；不假定本 checkout 已跟踪远端。
- 每次写代码前必须阅读 [CPP_STYLE](../../readme/框架/CPP_STYLE.md)、[SYSTEM_CONVENTIONS](../../readme/框架/SYSTEM_CONVENTIONS.md)，并查阅 [Google C++ 指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/) 中相关完整条款。
- 自有代码采用 C++，当前 CMake 声明 C++20。Google C++ 风格加项目补充：禁止自定义业务宏和全局变量，枚举值使用大写下划线，常量与变量同风格命名，类成员以下划线结尾；具体边界以 CPP_STYLE 为准。
- 使用本仓库 `.clang-format`，只格式化明确选中的自有文件。CubeMX、HAL、CMSIS、FreeRTOS、LibXR/XRobot 及自动生成模块保持上游接口和风格，不全仓重排、不将 C 文件直接改后缀。
- 保留用户修改；没有授权不提交、推送、烧录或驱动电机。有实质进展后更新同一集中状态。
