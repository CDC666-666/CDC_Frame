"""Run fresh ARM builds and real YAML incremental-build regressions."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]


# 作用运行实际构建命令并核对退出状态是否符合预期。
# 输入arguments 和 should_pass，输出打印的构建日志并返回日志文本，退出状态不符则断言失败。
def run(arguments, *, should_pass=True):
    result = subprocess.run(arguments, cwd=ROOT, text=True, encoding="utf-8", errors="replace",
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(result.stdout, end="", flush=True)
    if (result.returncode == 0) != should_pass:
        raise AssertionError(f"unexpected exit {result.returncode}: {arguments}")
    return result.stdout


# 作用计算待比较构建产物的 SHA-256。
# 输入文件 path，输出十六进制摘要字符串，读取失败抛出文件异常。
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


# 作用执行三个全新 ARM 构建及 YAML 增量、错误阻断和恢复验证。
# 输入命令行 --cmake 和 --ninja，输出构建目录、日志及断言结果，正常无返回值。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmake", required=True)
    parser.add_argument("--ninja", required=True)
    args = parser.parse_args()
    build = Path(tempfile.mkdtemp(prefix="phase1-clean-", dir=ROOT / "build"))
    source = ROOT / "01_assembly/config/frame.yaml"
    fixture = build / "frame.yaml"
    fixture.write_bytes(source.read_bytes())
    print(f"Fresh build root {build}", flush=True)
    directories = {}
    for preset in ("Debug-imu", "Debug-can-imu", "Debug-can-motor"):
        directory = build / preset
        directories[preset] = directory
        run([args.cmake, "--preset", preset, "-B", str(directory),
             f"-DCMAKE_MAKE_PROGRAM={args.ninja}", f"-DPython3_EXECUTABLE={sys.executable}",
             f"-DFRAME_CDC_CONFIG={fixture}"])
        run([args.cmake, "--build", str(directory), "-j", "8"])
        assert (directory / "generated/frame_config.c").is_file()
        commands = json.loads((directory / "compile_commands.json").read_text(encoding="utf-8"))
        for command in commands:
            normalized = command["file"].replace("\\", "/")
            assert not any(f"/FA/{old}/" in normalized for old in ("app", "base", "algorithm"))
        print(f"PASS fresh {preset}", flush=True)

    directory = directories["Debug-can-motor"]
    generated = directory / "generated/frame_config.c"
    old_generated = digest(generated)
    old_elf = digest(directory / "MC02.elf")
    data = yaml.safe_load(fixture.read_text(encoding="utf-8"))
    data["controller"]["fixed_speed_rad_s"] = 0.75
    fixture.write_text(yaml.safe_dump(data), encoding="utf-8")
    output = run([args.cmake, "--build", str(directory), "-j", "8"])
    assert "Validating YAML" in output
    assert ".fixed_speed_rad_s = 0.75" in generated.read_text(encoding="utf-8")
    assert digest(generated) != old_generated and digest(directory / "MC02.elf") != old_elf
    print("PASS YAML change regenerated configuration and changed linked ELF", flush=True)

    data["controller"]["control_period_us"] = 0
    fixture.write_text(yaml.safe_dump(data), encoding="utf-8")
    output = run([args.cmake, "--build", str(directory)], should_pass=False)
    assert "controller.control_period_us" in output and str(fixture) in output
    assert not generated.exists()
    print("PASS invalid YAML blocked the build and removed stale generated C", flush=True)

    fixture.write_bytes(source.read_bytes())
    run([args.cmake, "--build", str(directory), "-j", "8"])
    generated.unlink()
    run([args.cmake, "--build", str(directory), "-j", "8"])
    assert generated.exists()
    output = run([args.cmake, "--build", str(directory)])
    assert "no work to do" in output
    print("PASS restored configuration, missing generated-file recovery and no-op incremental build",
          flush=True)

    data = yaml.safe_load(fixture.read_text(encoding="utf-8"))
    data["scenes"]["motor_bench_motor"]["telemetry"] = "imu"
    fixture.write_text(yaml.safe_dump(data), encoding="utf-8")
    run([args.cmake, "--build", str(directory), "-j", "8"])
    commands = json.loads((directory / "compile_commands.json").read_text(encoding="utf-8"))
    files = [command["file"] for command in commands]
    assert any(path.endswith("/imu_telemetry.c") for path in files)
    assert not any(path.endswith("/motor_telemetry.c") for path in files)
    print(f"PASS YAML scene features automatically updated source selection. Artifacts {build}")


if __name__ == "__main__":
    main()
