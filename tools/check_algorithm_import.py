"""检查算法迁入文件是否仍与批准参考的哈希完全一致。"""

import hashlib
import json
from pathlib import Path


def main():
    """核对来源清单；读取本仓库22份文件，发现差异时返回非零。"""
    project = Path(__file__).resolve().parents[1]
    user_file = project / "User_File"
    manifest = json.loads((project / "tools/reference_algorithms.json").read_text(encoding="utf-8"))
    root = user_file / "1_Middleware/Algorithm"
    expected = {entry["path"]: entry["sha256"] for entry in manifest["files"]}
    actual = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in root.rglob("*") if path.is_file()}
    differences = sorted(name for name in expected.keys() | actual.keys()
                         if expected.get(name) != actual.get(name))
    forbidden = sorted(path.relative_to(user_file).as_posix()
                       for path in user_file.rglob("*") if path.is_file()
                       and (path.name.casefold().startswith("readme") or path.name in
                            (".gitattributes", "SOURCE_MANIFEST.json")))
    print(json.dumps({"reference_files": len(expected), "differences": differences,
                      "forbidden_user_file_metadata": forbidden}, indent=2))
    if len(expected) != 22 or differences or forbidden:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
