"""Host checks for YAML rejection, generated values and stale-output removal."""
import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True

import yaml

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "tools/generate_config.py"
spec = importlib.util.spec_from_file_location("generate_config", GENERATOR)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SOURCE = ROOT / "01_assembly/config/frame.yaml"


# 作用验证 YAML 错误拒绝、旧输出清除和生成 C 配置的真实数值。
# 输入命令行提供构建目录、主机编译器及包含路径，输出断言和通过信息，正常无返回值。
def main():
    good = module.load_config(SOURCE, "imu")
    cases = []

    # 作用复制基准配置并替换指定路径的字段供正负例使用。
    # 输入点分隔 path 和 value，输出修改后的深拷贝配置，基准 good 保留。
    def changed(path, value):
        data = copy.deepcopy(good)
        target = data
        for part in path.split(".")[:-1]:
            target = target[part]
        target[path.split(".")[-1]] = value
        return data

    for path, value in (
        ("imu.sample_period_us", True),
        ("imu.sample_period_us", 0),
        ("imu.sample_period_us", 25001),
        ("imu.sample_period_us", "5000us"),
        ("telemetry.period_us", -1),
        ("telemetry.period_us", 2**64),
        ("schema_version", 2),
        ("can.bitrate_hz", 333333),
        ("can.bitrate_hz", 1000),
        ("motor.motor_id", 0),
        ("motor.master_id", good["motor"]["motor_id"]),
        ("motor.position_max_rad", -1.0),
        ("motor.speed_max_rad_s", float("inf")),
        ("motor.feedback_timeout_us", 0),
        ("controller.control_period_us", 50000),
        ("controller.fixed_speed_rad_s", float("nan")),
        ("controller.fixed_speed_rad_s", 2.0),
        ("controller.mode", "arbitrary_expression()"),
        ("execution.enable_timeout_us", 10000),
        ("execution.disable_retry_us", 1),
        ("execution.kd_n_m_s_per_rad", 0.0),
        ("execution.has_status_request", 1),
        ("algorithm.pitch.pitch_deadband_rad", -0.1),
        ("algorithm.pitch.pitch_to_speed_per_s", 1.7e308),
        ("algorithm.trajectory.accel_limit_rad_s2", 0.0),
        ("algorithm.trajectory.speed_limit_rad_s", float("-inf")),
        ("algorithm.trajectory.accel_limit_rad_s2", 10**400),
        ("scenes.imu.telemetry", "motor"),
        ("scenes.imu.application", "ros2"),
    ):
        cases.append((path, yaml.safe_dump(changed(path, value))))
    data = copy.deepcopy(good)
    del data["motor"]["motor_id"]
    cases.append(("motor.motor_id", yaml.safe_dump(data)))
    data = copy.deepcopy(good)
    data["controller"]["fixed_speed_rpm"] = 100
    cases.append(("controller.fixed_speed_rpm", yaml.safe_dump(data)))
    cases.append(("schema_version", SOURCE.read_text(encoding="utf-8") + "\nschema_version: 1\n"))
    cases.append(("duplicate field mode",
                  SOURCE.read_text(encoding="utf-8").replace("  mode: feedback", "  mode: feedback\n  mode: feedback")))
    cases.append(("constructor", "!!python/object/apply:os.system ['echo prohibited']"))
    motion = changed("controller.mode", "fixed_speed")
    motion["can"]["bitrate_hz"] = 500000
    motion["motor"].update(position_max_rad=12.5, speed_max_rad_s=10.0, torque_max_n_m=6.0)
    module.validate(motion, "motor_bench_motor")
    for field in ("position_max_rad", "speed_max_rad_s", "torque_max_n_m"):
        data = copy.deepcopy(motion)
        data["motor"][field] = None
        cases.append((f"motor.{field}", yaml.safe_dump(data)))
    data = copy.deepcopy(motion)
    data["can"]["bitrate_hz"] = None
    cases.append(("can.bitrate_hz", yaml.safe_dump(data)))
    data = copy.deepcopy(motion)
    data["motor"]["speed_max_rad_s"] = 0.5
    cases.append(("algorithm.trajectory.speed_limit_rad_s", yaml.safe_dump(data)))

    # Use motor scene for device-configuration failures, so compatibility errors do not mask them.
    with tempfile.TemporaryDirectory(dir=sys.argv[1], prefix="yaml-") as directory:
        path = Path(directory) / "case.yaml"
        output = Path(directory) / "frame_config.c"
        for field, text in cases:
            path.write_text(text, encoding="utf-8")
            output.write_text("stale output", encoding="utf-8")
            scene = "imu" if field.startswith("scenes.") else "motor_bench_motor"
            result = subprocess.run([sys.executable, str(GENERATOR), "--config", str(path),
                                     "--scene", scene, "--output", str(output)],
                                    capture_output=True, text=True, encoding="utf-8")
            assert result.returncode != 0, field
            assert str(path) in result.stderr and field in result.stderr, (field, result.stderr)
            assert not output.exists(), field

        for scene in good["scenes"]:
            generated = module.generate(module.validate(copy.deepcopy(good), scene), scene)
            assert ".mode = SPEED_CONTROLLER_FEEDBACK" in generated
            assert ".bitrate_hz = 0," in generated
        altered = copy.deepcopy(good)
        altered["controller"]["fixed_speed_rad_s"] = 0.75
        first = module.generate(good, "imu")
        second = module.generate(module.validate(altered, "imu"), "imu")
        assert first != second and ".fixed_speed_rad_s = 0.75" in second

        path.write_text(yaml.safe_dump(altered), encoding="utf-8")
        subprocess.run([sys.executable, str(GENERATOR), "--config", str(path),
                        "--scene", "imu", "--output", str(output)], check=True)
        probe = Path(directory) / "probe.c"
        probe.write_text('#include "frame_config.h"\n'
                         'int main(void) { return g_frame_config.controller.fixed_speed_rad_s != 0.75; }\n',
                         encoding="utf-8")
        executable = Path(directory) / "probe.exe"
        subprocess.run([sys.argv[2], "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *["-I" + include for include in sys.argv[3:]],
                        str(probe), str(output), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)

        try:
            module.validate(good, "unknown")
        except module.ConfigError as error:
            assert "scenes.unknown" in str(error)
        else:
            raise AssertionError("unknown scene accepted")
    print(f"YAML passed {len(cases)} rejection cases, unknown scene, motion setup and changed output")


if __name__ == "__main__":
    main()
