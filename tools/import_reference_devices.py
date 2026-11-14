"""从唯一参考导入设备，显式改为实例依赖；不得用于覆盖人工修改后的目标。"""

import argparse
import hashlib
import json
from pathlib import Path
import re


def block_end(text, start):
    """寻找代码块终点；输入左花括号位置，返回对应右括号之后位置。"""
    depth = 1
    index = start + 1
    while depth:
        depth += (text[index] == "{") - (text[index] == "}")
        index += 1
    return index


def transform(text, name):
    """改写硬件依赖边界；输入参考文本和路径，输出保留协议的适配文本。"""
    text = re.sub(r"^(?:extern )?Class_\w+ BSP_\w+;\s*$", "", text, flags=re.M)
    text = text.replace("SYS_Timestamp.", "Class_Timestamp::")
    for prefix, parameter in [("CAN", "hcan"), ("UART", "huart")]:
        original = "FDCAN" if prefix == "CAN" else "UART"
        text = text.replace(f"const {original}_HandleTypeDef *{parameter}",
                            f"Struct_{prefix}_Manage_Object *{parameter}")
        # 保留分组分配算法，去掉三个总线重复分支，缓冲由传入对象拥有。
        marker = "uint8_t *allocate_tx_data("
        if prefix == "CAN" and marker in text:
            function = text.index(marker)
            opening = text.index("{", function)
            end = block_end(text, opening)
            bus = text.index("if (hcan == &hfdcan1)", opening)
            inner = text.index("{", bus)
            inner_end = block_end(text, inner)
            body = text[inner + 1:inner_end - 1]
            for group in ["1fe", "1ff", "200", "2fe", "2ff", "3fe", "4fe"]:
                body = body.replace(f"CAN1_0x{group}_Tx_Data", f"hcan->Group(0x{group})")
            text = text[:opening] + "{\n    uint8_t *tmp_tx_data_ptr = nullptr;\n" + body + "\n    return tmp_tx_data_ptr;\n}" + text[end:]
        pattern = rf"    if \({parameter}->Instance == [^\n]+\)\s*\{{\s*{prefix}_Manage_Object = &{prefix}1_Manage_Object;\s*\}}(?:\s*else if \([^\n]+\)\s*\{{\s*{prefix}_Manage_Object = &{prefix}\d+_Manage_Object;\s*\}})+"
        text = re.sub(pattern, f"    {prefix}_Manage_Object = {parameter};", text)
        text = text.replace(f"{prefix}_Manage_Object->{prefix}_Handler", f"{prefix}_Manage_Object")
    for prefix in ["SPI", "OSPI"]:
        text = text.replace(f"{prefix}_Manage_Object->{prefix}_Handler", f"{prefix}_Manage_Object")
    if "/Plotter/" in name and "USB" in text:
        text = text.replace("USB_Transmit_Data(", "USB_Transmit_Data(USB_Manage_Object, ")
        text = re.sub(r"(void (?:Class_\w+_USB::)?Init\()", r"\1Struct_USB_Manage_Object *driver, ", text)
        text = text.replace("USB_Manage_Object = &USB0_Manage_Object;", "USB_Manage_Object = driver;")
    contexts = {
        "BSP/WS2812/bsp_ws2812": ("SPI", "SPI6"),
        "BSP/Power/bsp_power": ("ADC", "ADC1"),
        "BSP/W25Q64JV/bsp_w25q64jv": ("OSPI", "OSPI2"),
        "BSP/BMI088/Accel/bsp_bmi088_accel": ("SPI", "SPI2"),
        "BSP/BMI088/Gyro/bsp_bmi088_gyro": ("SPI", "SPI2"),
        "BSP/BMI088/bsp_bmi088": ("SPI", "SPI2"),
    }
    for stem, (prefix, instance) in contexts.items():
        if name.rsplit(".", 1)[0].endswith(stem):
            text = re.sub(r"(void (?:Class_\w+::)?Init\()", rf"\1Struct_{prefix}_Manage_Object *driver, ", text)
            text = text.replace("driver, )", "driver)")
            text = text.replace(f"{prefix}_Manage_Object = &{instance}_Manage_Object;", f"{prefix}_Manage_Object = driver;")
    if name.endswith("BSP/BMI088/bsp_bmi088.cpp"):
        text = text.replace("BMI088_Accel.Init(true)", "BMI088_Accel.Init(driver, false)").replace("BMI088_Gyro.Init()", "BMI088_Gyro.Init(driver)")
    if name.endswith("BSP/Power/bsp_power.h"):
        start = text.index("    uint32_t adc_value")
        end = text.index("    return (voltage);", start)
        text = text[:start] + "    float voltage = ADC_Manage_Object->ReadVoltage() * 11.0f;\n" + text[end:]
    if name.endswith("BSP/W25Q64JV/bsp_w25q64jv.cpp"):
        text = text.replace("HAL_OSPI_MemoryMapped(OSPI_Manage_Object, &tmp_config)", "OSPI_Manage_Object->Map(true)")
    return text


def main():
    """导入参考设备；读取显式源目录，只创建缺失文件并生成工具区来源清单。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    source = args.source / "User_File"
    files = []
    for path in sorted((source / "2_Device").rglob("*")):
        if path.suffix not in (".cpp", ".h"):
            continue
        relative = path.relative_to(source)
        target = project / "User_File" / relative
        if target.exists():
            raise RuntimeError(f"Refusing to overwrite {target}")
        target.parent.mkdir(parents=True, exist_ok=True)
        original = path.read_bytes()
        content = transform(original.decode("utf-8-sig"), relative.as_posix())
        target.write_text(content, encoding="utf-8", newline="\n")
        files.append({"path": relative.as_posix(), "source_sha256": hashlib.sha256(original).hexdigest()})
    (project / "tools/reference_devices.json").write_text(json.dumps({"source": str(args.source), "files": files}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Imported {len(files)} device files")


if __name__ == "__main__":
    main()
