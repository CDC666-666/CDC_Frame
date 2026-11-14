"""在隔离目录运行 CubeMX，再保留用户区并生成本项目的 LibXR/XRobot 接入。"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

from generate_libxr import generate


def run(arguments, project, log=None):
    """检查外部命令结果；输入参数、工作目录及可选日志，失败时抛出异常。"""
    if log is None:
        subprocess.run(arguments, cwd=project, check=True)
        return
    with log.open("w", encoding="utf-8") as stream:
        result = subprocess.run(arguments, cwd=project, stdout=stream,
                                stderr=subprocess.STDOUT, check=False)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}); see {log}")


def preserve_user_regions(previous, generated):
    """保留 CubeMX 用户区；输入原文和新生成文本，返回合并结果。"""
    pattern = r"(/\* USER CODE BEGIN ([^\r\n*]+) \*/)(.*?)(/\* USER CODE END \2 \*/)"
    blocks = {match.group(2): match.group(3)
              for match in re.finditer(pattern, previous, re.S)}
    return re.sub(pattern, lambda match: match.group(1) +
                  blocks.get(match.group(2), match.group(3)) + match.group(4),
                  generated, flags=re.S)


def normalize_cmake(text, generated_root):
    """整理生成路径和重复源条目；输入 CMake 文本及生成根，返回可搬移文本。"""
    text = text.replace(generated_root.as_posix() + "/", "${CMAKE_CURRENT_SOURCE_DIR}/../../")
    text = text.replace("../../startup_stm32h723xx.s",
                        "../../Core/Startup/startup_stm32h723vgtx.s")

    def unique_sources(match):
        """去除同一源列表中的重复行；输入列表匹配，返回稳定的源列表。"""
        seen = set()
        lines = []
        for line in match.group(2).splitlines():
            if line.strip() and line.strip() in seen:
                continue
            seen.add(line.strip())
            lines.append(line)
        return match.group(1) + "\n".join(lines) + match.group(3)

    text = re.sub(r"(set\(MX_Application_Src\s*\n)(.*?)(\n\))", unique_sources, text, flags=re.S)
    return "\n".join(line.rstrip() for line in text.splitlines()) + "\n"


def normalize_ioc(text):
    """移除本机固件路径；输入生成 IOC，返回单一、可迁移配置源。"""
    entries = dict(line.split("=", 1) for line in text.splitlines()
                   if "=" in line and not line.startswith("#"))
    entries.update({"ProjectManager.CustomerFirmwarePackage": "",
                    "ProjectManager.DefaultFWLocation": "true",
                    "ProjectManager.UnderRoot": "true",
                    "ProjectManager.DeletePrevious": "false"})
    return "#MicroXplorer Configuration settings - do not modify\n" + "\n".join(
        f"{key}={entries[key]}" for key in sorted(entries)) + "\n"


def merge_cubemx(project, generated):
    """应用已生成的基础层；输入工程和隔离生成根，保留用户区及现有 SDK。"""
    kernel = generated / "Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
    if '"V10.6.2"' not in kernel.read_text(encoding="utf-8-sig"):
        raise RuntimeError("Unexpected FreeRTOS kernel; expected V10.6.2")
    hal = generated / "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
    if not re.search(r"VERSION_SUB2\s+\(0x06UL\)", hal.read_text(encoding="utf-8-sig")):
        raise RuntimeError("Unexpected HAL version; expected 1.11.6 from FW_H7 V1.13.0")
    for relative in ("Core/Inc", "Core/Src"):
        for source in sorted((generated / relative).iterdir()):
            if not source.is_file():
                continue
            destination = project / relative / source.name
            text = source.read_text(encoding="utf-8-sig")
            if destination.exists():
                text = preserve_user_regions(destination.read_text(encoding="utf-8-sig"), text)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(text, encoding="utf-8", newline="\n")
    # SDK 原件保持不变，只补充新外设所需、之前不存在的同版本文件。
    for relative in ("Drivers", "Middlewares/Third_Party/FreeRTOS"):
        for source in sorted((generated / relative).rglob("*")):
            if source.is_file():
                destination = project / source.relative_to(generated)
                if not destination.exists():
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, destination)
    startup = project / "Core/Startup/startup_stm32h723vgtx.s"
    startup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(generated / "startup_stm32h723xx.s", startup)
    cmake = generated / "cmake/stm32cubemx/CMakeLists.txt"
    (project / "cmake/stm32cubemx/CMakeLists.txt").write_text(
        normalize_cmake(cmake.read_text(encoding="utf-8-sig"), generated),
        encoding="utf-8", newline="\n")
    (project / "dm02_test.ioc").write_text(
        normalize_ioc((generated / "dm02_test.ioc").read_text(encoding="utf-8-sig")),
        encoding="utf-8", newline="\n")
    metadata = generated / ".mxproject"
    if metadata.is_file():
        text = metadata.read_text(encoding="utf-8-sig")
        text = text.replace(str(generated) + "\\", "")
        text = text.replace(generated.as_posix() + "/", "")
        text = text.replace("startup_stm32h723xx.s", "Core/Startup/startup_stm32h723vgtx.s")
        text = text.replace("STM32H723XG_FLASH.ld", "STM32H723VGTX_FLASH.ld")
        (project / ".mxproject").write_text(text, encoding="utf-8", newline="\n")
    # 现有链接脚本是运行适配文件，保留 DTCM/TLS、DMA 段和原有堆栈布局。
    if not (project / "STM32H723VGTX_FLASH.ld").is_file():
        raise RuntimeError("The LibXR-compatible linker script must be preserved")


def cubemx_generate(project, cubemx, firmware):
    """在工程 build 内隔离生成；输入工具及固件路径，合并限定的生成层文件。"""
    cubemx = cubemx.resolve()
    firmware = firmware.resolve()
    if not cubemx.is_file() or not (firmware / "Drivers").is_dir():
        raise RuntimeError("Provide an existing CubeMX executable and FW_H7 V1.13.0 directory")
    parent = (project / "build/regenerate").resolve()
    parent.mkdir(parents=True, exist_ok=True)
    stage = parent / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    stage.mkdir()
    if not stage.resolve().is_relative_to(parent):
        raise RuntimeError("Generation directory escaped the build directory")
    shutil.copy2(project / "dm02_test.ioc", stage / "dm02_test.ioc")
    script = stage / "generate.mxscript"
    script.write_text(
        f"config load {stage.as_posix()}/dm02_test.ioc\n"
        f"project path {stage.as_posix()}\nproject name dm02_test\n"
        "project generateunderroot 1\nproject couplefilesbyip 1\n"
        "project toolchain CMake\nproject compiler GCC\n"
        f"project setCustomFWPath {firmware.as_posix()}\n"
        "project generate\nconfig save\nexit\n", encoding="utf-8")
    java = cubemx.parent / "jre/bin/java.exe"
    log = stage / "cubemx.log"
    run([str(java), "-jar", str(cubemx), "-q", str(script)], cubemx.parent, log)
    generated = stage if (stage / "Core/Src/main.c").exists() else stage / "dm02_test"
    if not (generated / "Core/Src/main.c").is_file():
        raise RuntimeError(f"CubeMX did not generate the expected project; see {log}")
    merge_cubemx(project, generated)
    print(f"CubeMX generation log: {log}")


def main():
    """运行可复现生成链；读取工具路径或仅 LibXR 模式，更新本工程生成文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--cubemx", type=Path)
    parser.add_argument("--firmware-root", type=Path)
    parser.add_argument("--libxr-only", action="store_true")
    parser.add_argument("--offline", action="store_true", help="Use already restored XRobot modules")
    args = parser.parse_args()
    project = args.project.resolve()
    if not args.libxr_only:
        if args.cubemx is None or args.firmware_root is None:
            parser.error("CubeMX generation needs --cubemx and --firmware-root")
        cubemx_generate(project, args.cubemx, args.firmware_root)
    run([sys.executable, "-m", "libxr", "parse", "-d", str(project)], project)
    generate(project)
    run([sys.executable, "-m", "libxr", "stm32", "cmake", "-d", str(project)], project)
    xrobot = Path(sys.executable).with_name("xrobot.exe" if os.name == "nt" else "xrobot")
    command = [str(xrobot), "setup", "--frozen"]
    if args.offline:
        command.append("--offline")
    run(command, project)


if __name__ == "__main__":
    main()
