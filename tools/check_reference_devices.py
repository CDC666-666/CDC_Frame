"""核对功能目录覆盖及适配基线；可选核对本地原参考，不连接硬件。"""

import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    """计算文件摘要；输入路径，返回SHA-256。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """校验48份适配及2份批准移除项；输入参考根，输出来源/目标差异。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path,
                        help="唯一dm02_test参考根；不传时只核对仓库快照")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    root = project / "User_File"
    manifest = json.loads((project / "tools/reference_devices.json").read_text(encoding="utf-8"))
    expected = {entry["path"]: entry for entry in manifest["files"]}
    retired = {entry["path"]: entry for entry in manifest.get("retired_files", [])}
    actual = {path.relative_to(root).as_posix() for path in root.rglob("*")
              if path.is_file() and not any(path.is_relative_to(root / folder)
                  for folder in ("1_Middleware/Algorithm", "3_Modules", "4_Task/User", "4_Task/rm_music"))}
    differences = [name for name in sorted(actual | expected.keys())
                   if name not in expected or name not in actual or
                   digest(root / name) != expected[name]["adapted_sha256"]]
    source_differences = []
    if args.reference:
        source = args.reference / "User_File"
        source_differences = [name for name, entry in (expected | retired).items()
                              if not (source / name).is_file() or
                              digest(source / name) != entry["source_sha256"]]
    result = {"adapted_files": len(actual), "differences": differences,
              "retired_files": sorted(retired),
              "source_checked": args.reference is not None,
              "source_differences": source_differences}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    approved_retired = {"4_Task/tsk_config_and_callback.cpp", "4_Task/tsk_config_and_callback.h"}
    if (len(expected) != 48 or len(actual) != 48 or set(retired) != approved_retired
            or expected.keys() & retired.keys() or differences or source_differences):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
