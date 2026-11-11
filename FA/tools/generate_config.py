"""Validate phase-one YAML and emit data-only C configuration."""

import argparse
import copy
import math
from pathlib import Path
import sys

try:
    import yaml
except ImportError:
    raise SystemExit("PyYAML is required. Install tools/requirements.txt for this Python interpreter.")


class ConfigError(ValueError):
    pass


class UniqueSafeLoader(yaml.SafeLoader):
    """SafeLoader with duplicate-key rejection, including merged mappings."""

    # 作用安全构造 YAML 映射并拒绝非字符串键和重复键。
    # 输入self 加载器、node 和 deep，输出映射字典，键非法时抛出 ConfigError。
    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if type(key) is not str:
                raise ConfigError(f"line {key_node.start_mark.line + 1}: mapping key must be a string")
            if key in result:
                raise ConfigError(f"line {key_node.start_mark.line + 1}: duplicate field {key}")
            result[key] = self.construct_object(value_node, deep=deep)
        return result


# 作用用字段路径构造配置校验异常。
# 输入field 和 message，输出抛出的 ConfigError，无正常返回值。
def fail(field, message):
    raise ConfigError(f"{field}: {message}")


# 作用检查映射类型、未知字段和缺失字段。
# 输入value、field 和必填 keys，输出原映射，校验失败抛出 ConfigError。
def mapping(value, field, keys):
    if type(value) is not dict:
        fail(field, "expected mapping")
    unknown = value.keys() - set(keys)
    missing = set(keys) - value.keys()
    if unknown:
        fail(f"{field}.{sorted(unknown)[0]}", "unknown field")
    if missing:
        fail(f"{field}.{sorted(missing)[0]}", "required field missing")
    return value


# 作用检查数值类型、有限性、范围及允许的空值。
# 输入value、field、上下界及 integer/nullable，输出无，失败抛出 ConfigError。
def number(value, field, minimum, maximum, *, integer=False, nullable=False):
    if nullable and value is None:
        return
    expected = (int,) if integer else (int, float)
    if type(value) not in expected:
        fail(field, "expected integer" if integer else "expected number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or not minimum <= value <= maximum:
        fail(field, f"expected finite value in [{minimum}, {maximum}]")


# 作用检查配置值是否为允许的字符串枚举。
# 输入value、field 和 choices，输出无，失败抛出 ConfigError。
def choice(value, field, choices):
    if type(value) is not str or value not in choices:
        fail(field, f"expected one of {', '.join(choices)}")


# 作用检查 YAML 结构、参数范围和所选场景的跨字段兼容性。
# 输入data 和 scene，输出通过校验的原 data，失败抛出 ConfigError。
def validate(data, scene):
    root_keys = ("schema_version", "scenes", "imu", "telemetry", "can",
                 "motor", "execution", "controller", "algorithm")
    mapping(data, "$", root_keys)
    number(data["schema_version"], "schema_version", 1, 1, integer=True)
    scenes = data["scenes"]
    if type(scenes) is not dict or not scenes:
        fail("scenes", "expected nonempty mapping")
    for name, features in scenes.items():
        if type(name) is not str or not name.isascii() or not name.replace("_", "").isalnum():
            fail("scenes", "names must contain ASCII letters, digits or underscores")
        mapping(features, f"scenes.{name}", ("application", "telemetry"))
        choice(features["application"], f"scenes.{name}.application", ("imu", "motor_bench"))
        choice(features["telemetry"], f"scenes.{name}.telemetry", ("imu", "motor"))
        if features["telemetry"] == "motor" and features["application"] != "motor_bench":
            fail(f"scenes.{name}.telemetry", "motor output requires motor_bench application")
    if scene not in scenes:
        fail(f"scenes.{scene}", "unknown selected scene")

    imu = mapping(data["imu"], "imu", ("sample_period_us",))
    # Existing attitude estimator rejects steps above 0.025 s.
    number(imu["sample_period_us"], "imu.sample_period_us", 1, 25000, integer=True)
    telemetry = mapping(data["telemetry"], "telemetry", ("period_us",))
    number(telemetry["period_us"], "telemetry.period_us", 1, 2**63 - 1, integer=True)
    bus = mapping(data["can"], "can", ("bitrate_hz",))
    number(bus["bitrate_hz"], "can.bitrate_hz", 1, 1000000, integer=True, nullable=True)
    bitrate = bus["bitrate_hz"]
    # MC02 uses 24 MHz / (24 TQ * prescaler), prescaler in 1..512.
    if bitrate is not None and (1000000 % bitrate != 0 or 1000000 // bitrate > 512):
        fail("can.bitrate_hz", "unsupported by current MC02 FDCAN timing")

    motor = mapping(data["motor"], "motor", ("motor_id", "master_id", "position_max_rad",
                      "speed_max_rad_s", "torque_max_n_m", "feedback_timeout_us"))
    number(motor["motor_id"], "motor.motor_id", 1, 0x7FE, integer=True)
    number(motor["master_id"], "motor.master_id", 0, 0x7FE, integer=True)
    if motor["motor_id"] == motor["master_id"]:
        fail("motor.master_id", "must differ from motor_id")
    for key in ("position_max_rad", "speed_max_rad_s", "torque_max_n_m"):
        number(motor[key], f"motor.{key}", sys.float_info.min, sys.float_info.max / 2,
               nullable=True)
    number(motor["feedback_timeout_us"], "motor.feedback_timeout_us", 1, 2**63 - 1, integer=True)

    execution = mapping(data["execution"], "execution",
                        ("has_status_request", "enable_timeout_us", "status_period_us",
                         "disable_retry_us", "kd_n_m_s_per_rad"))
    if type(execution["has_status_request"]) is not bool:
        fail("execution.has_status_request", "expected boolean")
    for key in ("enable_timeout_us", "status_period_us"):
        number(execution[key], f"execution.{key}", 1, 2**63 - 1, integer=True)
    number(execution["disable_retry_us"], "execution.disable_retry_us", 2, 2**63 - 1, integer=True)
    number(execution["kd_n_m_s_per_rad"], "execution.kd_n_m_s_per_rad", 5.0 / 4095.0, 5.0)

    controller = mapping(data["controller"], "controller",
                         ("mode", "control_period_us", "attitude_timeout_us", "fixed_speed_rad_s"))
    choice(controller["mode"], "controller.mode", ("feedback", "fixed_speed", "pitch_speed"))
    for key in ("control_period_us", "attitude_timeout_us"):
        number(controller[key], f"controller.{key}", 1, 2**63 - 1, integer=True)
    number(controller["fixed_speed_rad_s"], "controller.fixed_speed_rad_s",
           -sys.float_info.max, sys.float_info.max)
    if controller["control_period_us"] >= motor["feedback_timeout_us"]:
        fail("controller.control_period_us", "must be less than motor.feedback_timeout_us")
    if controller["control_period_us"] >= execution["enable_timeout_us"]:
        fail("execution.enable_timeout_us", "must exceed controller.control_period_us")

    algorithm = mapping(data["algorithm"], "algorithm", ("pitch", "trajectory"))
    pitch = mapping(algorithm["pitch"], "algorithm.pitch",
                    ("center_pitch_rad", "pitch_deadband_rad", "pitch_to_speed_per_s"))
    number(pitch["center_pitch_rad"], "algorithm.pitch.center_pitch_rad", -math.pi / 2, math.pi / 2)
    number(pitch["pitch_deadband_rad"], "algorithm.pitch.pitch_deadband_rad", 0, math.pi)
    number(pitch["pitch_to_speed_per_s"], "algorithm.pitch.pitch_to_speed_per_s",
           -sys.float_info.max, sys.float_info.max)
    trajectory = mapping(algorithm["trajectory"], "algorithm.trajectory",
                         ("speed_limit_rad_s", "accel_limit_rad_s2", "decel_limit_rad_s2"))
    for key, value in trajectory.items():
        number(value, f"algorithm.trajectory.{key}", sys.float_info.min, sys.float_info.max)
    limit = trajectory["speed_limit_rad_s"]
    if abs(controller["fixed_speed_rad_s"]) > limit:
        fail("controller.fixed_speed_rad_s", "magnitude exceeds algorithm.trajectory.speed_limit_rad_s")
    for key in ("accel_limit_rad_s2", "decel_limit_rad_s2"):
        if not math.isfinite(trajectory[key] * (controller["control_period_us"] * 1e-6)):
            fail(f"algorithm.trajectory.{key}", "rate times control step must be finite")
    if not math.isfinite((math.pi / 2 + abs(pitch["center_pitch_rad"])) *
                         pitch["pitch_to_speed_per_s"]):
        fail("algorithm.pitch.pitch_to_speed_per_s", "mapping must remain finite")

    if controller["mode"] != "feedback":
        if scenes[scene]["application"] != "motor_bench":
            fail("controller.mode", "motion mode requires a motor_bench scene")
        for key in ("position_max_rad", "speed_max_rad_s", "torque_max_n_m"):
            if motor[key] is None:
                fail(f"motor.{key}", "motion mode requires verified range")
        if bitrate is None:
            fail("can.bitrate_hz", "motion mode requires verified bitrate")
        if limit > motor["speed_max_rad_s"]:
            fail("algorithm.trajectory.speed_limit_rad_s", "exceeds motor.speed_max_rad_s")
    return data


# 作用使用拒绝重复键的安全加载器读取并校验配置。
# 输入path 和 scene，输出配置字典，读取、解析或校验失败抛出对应异常。
def load_config(path, scene):
    with path.open(encoding="utf-8") as source:
        data = yaml.load(source, Loader=UniqueSafeLoader)
    return validate(data, scene)


# 作用把已验证的标量转换为确定性的 C 常量文本。
# 输入有限数字、布尔值或空值 value，输出对应 C 字面量字符串。
def c_value(value):
    if value is None:
        return "0"
    if type(value) is bool:
        return "true" if value else "false"
    if type(value) is int:
        return f"{value}ULL"
    result = format(value, ".17g")
    return result if "." in result or "e" in result else result + ".0"


# 作用递归把配置字典排版为 C 指定成员初始化文本。
# 输入data 和缩进宽度 indent，输出多行 C 初始化字符串。
def initializer(data, indent=4):
    lines = ["{"]
    for key, value in data.items():
        rendered = initializer(value, indent + 4) if type(value) is dict else c_value(value)
        lines.append(" " * indent + f".{key} = {rendered},")
    lines.append(" " * (indent - 4) + "}")
    return "\n".join(lines)


# 作用将已验证配置和场景转换为仅含 const 数据的 C 源码。
# 输入data 和 scene，输出带自动生成标记的 C 源码字符串。
def generate(data, scene):
    values = copy.deepcopy({key: data[key] for key in
                            ("imu", "telemetry", "can", "motor", "execution", "controller")})
    mode = values["controller"].pop("mode")
    values["controller"].update(data["algorithm"])
    body = initializer(values)
    # Enumeration tokens are selected by validation, never taken as C text from YAML.
    enum = {"feedback": "SPEED_CONTROLLER_FEEDBACK", "fixed_speed": "SPEED_CONTROLLER_FIXED_SPEED",
            "pitch_speed": "SPEED_CONTROLLER_PITCH_SPEED"}[mode]
    body = body.replace(".controller = {", f".controller = {{\n        .mode = {enum},", 1)
    return ('/* Automatically generated. Edit the selected YAML, not this file. */\n'
            f'/* Selected scene: {scene} */\n'
            '#include "frame_config.h"\n\n'
            f'const frame_config_t g_frame_config = {body};\n')


# 作用解析命令行并校验场景，按需输出特征或原子写入生成配置。
# 输入命令行的配置路径、场景和可选输出路径，输出特征或文件，成功返回 0，配置等错误返回 1。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        data = load_config(args.config, args.scene)
        if args.output is None:
            features = data["scenes"][args.scene]
            print(f'{features["application"]};{features["telemetry"]}')
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            # Write only after validation. On failure remove stale output below.
            temporary = args.output.with_suffix(".tmp")
            temporary.write_text(generate(data, args.scene), encoding="utf-8", newline="\n")
            temporary.replace(args.output)
    except (ConfigError, yaml.YAMLError, OSError) as error:
        if args.output is not None:
            args.output.unlink(missing_ok=True)
        print(f"{args.config}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
