"""复现原算法的模板接口、未初始化读取与数组越界，不修改被测源码。"""

import argparse
import json
import os
from pathlib import Path
import subprocess


def compile_probe(compiler, project, output, name, text, extra):
    """编译单个复现程序；输入工具、目录及源码，返回真实诊断和退出状态。"""
    source = output / (name + ".cpp")
    source.write_text(text, encoding="utf-8")
    artifact = output / (name + (".exe" if "-c" not in extra else ".o"))
    command = [str(compiler), "-std=c++20", "-I" + str(
        project / "User_File"), "-I" + str(
        project / "Drivers/CMSIS/DSP/Include"), "-I" + str(
        project / "Drivers/CMSIS/Include"), *extra, str(source), "-o", str(artifact)]
    result = subprocess.run(command, capture_output=True, text=True, errors="replace")
    (output / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
    return artifact, {"name": name, "compile_exit": result.returncode,
                      "command": command, "diagnostics": result.stdout + result.stderr}


def main():
    """执行可重复诊断；读取编译器及输出目录，写报告并对实际缺陷返回非零。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if os.name == "nt":
        import ctypes
        # 子进程继承当前进程的错误模式，越界 trap 直接退出，不弹 WER 对话框。
        ctypes.windll.kernel32.SetErrorMode(0x0001 | 0x0002)
    matrix = '#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"\n'
    frequency = '#include "1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.h"\n'
    queue = '#include "1_Middleware/Algorithm/Queue/alg_queue.h"\n'
    ekf = '#include "1_Middleware/Algorithm/Filter/EKF/alg_filter_ekf.h"\n'
    compile_cases = {
        "matrix_inequality": matrix + "int main() { Class_Matrix_f32<2,2> a,b; return a.template operator!=<2,2>(b); }\n",
        "const_matrix_product": matrix + "int main() { const Class_Matrix_f32<2,2> a,b; auto c=a*b; return 0; }\n",
        "ekf_unequal_dimensions": ekf + """
Class_Matrix_f32<1,1> H(const Class_Matrix_f32<2,1>&, const float&) { return {}; }
Class_Matrix_f32<1,2> J(const Class_Matrix_f32<2,1>&, const float&) { return {}; }
Class_Matrix_f32<1,1> V(const Class_Matrix_f32<2,1>&, const float&) { return {}; }
int main() { Class_Filter_EKF<2,1,1> f; f.Config_Nonlinear_Measurement_Model(H,J,V); }
""",
        "fir_unlimited_uninitialized": frequency + """
struct Probe : Class_Filter_Frequency<4> { float Input() { return Input_Signal[0]; } };
float Check() { Probe f; f.Init(0,0,Filter_Frequency_Type_LOWPASS,100,500,1000);
 f.Set_Now(1); return f.Input(); }
""",
        "queue_empty_uninitialized": queue + "int Check() { Class_Queue<int,3> q; q.Init(); return q.Pop(); }\n",
    }
    records = []
    for name, text in compile_cases.items():
        flags = ["-O2", "-Wall", "-Wextra", "-Werror=uninitialized",
                 "-Werror=maybe-uninitialized"]
        if name != "ekf_unequal_dimensions":
            flags.append("-c")
        _, record = compile_probe(args.compiler, project, output, name, text, flags)
        record["kind"] = "compile_or_uninitialized"
        records.append(record)
        print(f"{name}: compiler exit {record['compile_exit']}")
    # trap 模式不依赖本机是否安装 UBSan runtime；每例使用独立子进程。
    # 先验证合法下标可运行，再解释非法下标的 trap，避免把环境错误当成越界。
    fsm = '#include "1_Middleware/Algorithm/FSM/alg_fsm.h"\n'
    bounds_cases = {
        "bounds_control": fsm + "int main() { Class_FSM<2> f; f.Init(1); return 0; }\n",
        "fsm_init_out_of_range": fsm + "int main() { Class_FSM<2> f; f.Init(2); return 0; }\n",
        "fsm_transition_out_of_range": fsm + "int main() { Class_FSM<2> f; f.Init(0); f.Set_Status(2); return 0; }\n",
        "matrix_column_out_of_range": matrix + "int main() { Class_Matrix_f32<2,2> m; auto c=m.Get_Column(2); return int(c[1][0]); }\n",
    }
    for name, text in bounds_cases.items():
        artifact, record = compile_probe(args.compiler, project, output, name, text,
                                         ["-O0", "-fsanitize=bounds-strict",
                                          "-fsanitize-undefined-trap-on-error"])
        record["kind"] = "bounds_trap"
        if record["compile_exit"] == 0:
            try:
                result = subprocess.run([str(artifact)], capture_output=True, text=True,
                                        errors="replace", timeout=10)
                record.update({"run_exit": result.returncode,
                               "run_output": result.stdout + result.stderr})
            except subprocess.TimeoutExpired:
                record.update({"run_exit": None, "run_status": "timeout"})
        records.append(record)
        print(f"{name}: compiler exit {record['compile_exit']}, process exit {record.get('run_exit')}")
    report = {"compiler": str(args.compiler), "records": records}
    (output / "diagnostics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    control = next(record for record in records if record["name"] == "bounds_control")
    if control.get("run_exit") != 0:
        raise SystemExit("Bounds control failed: cannot interpret the crash probes")
    if any(record["compile_exit"] or record.get("run_exit", 0)
           or record.get("run_status") == "timeout" for record in records):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
