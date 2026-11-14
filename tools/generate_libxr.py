"""为参考 GPIO 标签生成合法的 LibXR C++ 对象名，保留 CubeMX/HAL 名称。"""

import argparse
from importlib.metadata import version
from pathlib import Path
import re

from libxr import generator_code_stm32


def generate(project):
    """生成硬件封装；输入工程根目录，输出保持参考 HAL 宏的 app_main。"""
    if version("libxr") != "6.0.1":
        raise RuntimeError("This compatibility adapter requires libxr==6.0.1")
    original_name = generator_code_stm32._gpio_object_name

    def gpio_object_name(port, gpio_data):
        """规范对象名；输入 GPIO 信息，输出合并连续下划线后的 C++ 名称。"""
        return re.sub(r"_+", "_", original_name(port, gpio_data))

    # 固定版本的适配点只影响 C++ 对象名；宏参数与生成器的冲突检查保持有效。
    # 不修改安装包，也不修改 .config.yaml 或参考 IOC 中的 GPIO 标签。
    generator_code_stm32._gpio_object_name = gpio_object_name
    try:
        generator_code_stm32.generate(
            str(project / ".config.yaml"),
            str(project / "User/app_main.cpp"),
            True,
            str(project / "User/libxr_config.yaml"),
        )
    finally:
        generator_code_stm32._gpio_object_name = original_name


def main():
    """运行固定版本生成适配；读取可选根目录参数，写入该工程的生成文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    arguments = parser.parse_args()
    generate(arguments.project.resolve())


if __name__ == "__main__":
    main()
