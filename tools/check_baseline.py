"""核对参考外设配置、RTOS 接入和明确批准的例外，不连接硬件。"""

import json
from pathlib import Path
import re


def read_ioc(path):
    """读取 IOC；输入配置路径，输出不含注释的键值表。"""
    return dict(line.split("=", 1) for line in path.read_text(encoding="utf-8-sig").splitlines()
                if "=" in line and not line.startswith("#"))


def main():
    """检查迁移契约；读取本工程源码和参考快照，失败时返回非零退出码。"""
    project = Path(__file__).resolve().parents[1]
    expected = json.loads((project / "tools/reference_config.json").read_text(encoding="utf-8"))
    ioc = read_ioc(project / "dm02_test.ioc")
    failures = []

    def require(condition, description):
        """登记验收条件；输入真假值及说明，失败项追加到本轮结果。"""
        if not condition:
            failures.append(description)

    for key, value in expected["peripheral_config"].items():
        require(ioc.get(key) == value, f"Reference mismatch: {key}: {ioc.get(key)} != {value}")
    require(len(list(project.glob("*.ioc"))) == 1, "Expected one root IOC")
    require(ioc["ProjectManager.FirmwarePackage"] == "STM32Cube FW_H7 V1.13.0", "Firmware package changed")
    require(ioc["NVIC.TimeBaseIP"] == "TIM17", "HAL timebase must be TIM17")
    require(ioc["NVIC.TIM17_IRQn"].split(r"\:")[1] == "0", "TIM17 priority must be 0")
    require(ioc["PA5.Signal"] == "SPI6_SCK", "PA5 must be SPI6 SCK")
    require(not any(value == "ADC_CHANNEL_19" for key, value in ioc.items() if key.startswith("ADC1.")), "ADC19 must be absent")
    for pin in ("PB10", "PB11", "PE15"):
        require(ioc.get(pin + ".Signal") == "GPIO_Output", f"LCD GPIO missing: {pin}")
    require(ioc.get("DAC1.DAC_Channel-DAC_OUT2_Int") == "DAC_CHANNEL_2", "DAC2 missing")
    require(ioc.get("Bdma.Request0") == "SPI6_TX", "SPI6 BDMA configuration missing")
    for key, value in ioc.items():
        if (key.startswith("NVIC.") and key.endswith("_IRQn") and value.startswith("true")
                and re.search(r"DMA|BDMA|MDMA|FDCAN|SPI|UART|USART|EXTI|OCTOSPI", key)):
            require(int(value.split(r"\:")[1]) >= 5, f"RTOS-facing IRQ priority: {key}")
    main_text = (project / "Core/Src/main.c").read_text(encoding="utf-8")
    freertos = (project / "Core/Src/freertos.c").read_text(encoding="utf-8")
    config = (project / "Core/Inc/FreeRTOSConfig.h").read_text(encoding="utf-8")
    app = (project / "User/app_main.cpp").read_text(encoding="utf-8")
    require("osKernelStart();" in main_text and "MX_FREERTOS_Init();" in main_text, "RTOS startup missing")
    require("app_main();" in freertos and "libxr_fatal_error" in freertos, "RTOS integration hooks missing")
    require(re.findall(r"^#define configUSE_TIMERS\s+(\d+)", config, re.M)[-1] == "0", "FreeRTOS software timers unexpectedly enabled")
    require("timebase(&htim17)" in app, "LibXR timebase not bound to TIM17")
    require("STM32Watchdog iwdg1(&hiwdg1, 8192, 1000)" in app, "Watchdog timing changed")
    require('"iwdg1_wdg", 1024, Thread::Priority::HIGH' in app, "Watchdog thread missing")
    require("spi6_tx_buf, UINT32_MAX" in app, "SPI6 must retain polling semantics")
    task = (project / "User_File/4_Task/tsk_config_and_callback.cpp").read_text(encoding="utf-8")
    require("XROBOT_MAIN();" in app and "terminal.ThreadFun" in task,
            "XRobot/routed terminal integration missing")
    require("term_thread.Create" not in app and "self->usb_.Process();" in task,
            "Physical CDC must have one application reader")
    require("hardware_.terminal.read_port_ = &console_input_;" in task,
            "Terminal must consume routed logical input")
    require("ADC_CHANNEL_19" not in app, "ADC19 still registered")
    require("X-CUBE-ALGOBUILD" not in (project / "dm02_test.ioc").read_text(encoding="utf-8"), "Unexpected DSP pack dependency")
    require(not (project / "USB_DEVICE").exists(), "Second USB device stack present")
    for relative in ("User_File/2_Device", "User_File/4_Task", "User_File/1_Middleware/Driver"):
        require((project / relative).is_dir(), f"Approved reference adaptation missing: {relative}")
    require("ReferenceApplication" in app, "Reference application owner missing")
    for directory in (project / "Core", project / "User", project / "User_File"):
        for path in directory.rglob("*"):
            if path.suffix in (".c", ".cpp", ".h", ".hpp"):
                text = path.read_text(encoding="utf-8-sig")
                require(not re.search(r"\bTask_(?:Init|Loop)\s*\(", text), f"Reference task imported: {path}")
    linker = (project / "STM32H723VGTX_FLASH.ld").read_text(encoding="utf-8")
    require("LENGTH = 768K" in linker, "Database sectors are not reserved from firmware")
    require(".axi_ram" in linker and ".ram_d3" in linker, "DMA memory sections missing")
    cmake = (project / "cmake/stm32cubemx/CMakeLists.txt").read_text(encoding="utf-8")
    require("Core/Startup/startup_stm32h723vgtx.s" in cmake, "Startup layout changed")
    require(not re.search(r"[A-Za-z]:[/\\]", cmake), "Machine-specific source path in CMake")
    result = {"reference_ioc_sha256": expected["source_ioc_sha256"],
              "reference_keys_checked": len(expected["peripheral_config"]), "failures": failures}
    destination = project / "build/baseline-validation.json"
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
