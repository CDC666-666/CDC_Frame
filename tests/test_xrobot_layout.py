"""验证真实官方生成器的依赖检查、产品选择及CMake过期拒绝行为。"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from project_layout import configure_xrobot, local_records

configure_xrobot()
from xrobot.generate_main import generate_code, load_modules, read_registrations, validate_all
from xrobot.project import Project, input_digest
from xrobot.describe import describe


class XRobotLayoutTest(unittest.TestCase):
    """使用项目真实模块和配置，检查迁移后生成行为。"""

    @classmethod
    def setUpClass(cls):
        """加载已锁模块及仓内模块，读取真实硬件注册。"""
        cls.project = Project.discover(ROOT)
        cls.modules = load_modules(cls.project)
        cls.registrations = read_registrations(cls.project.entry())

    def render(self, path):
        """输入配置路径，返回官方生成文本，不改选中的产品。"""
        return generate_code(self.project, path, self.modules, self.registrations)

    def test_product_selection(self):
        """按各产品当前开关装配音乐，旧服务及自带开机音不得再构造。"""
        self.assertEqual(validate_all(self.project, self.modules), 2)
        for path in (self.project.default_config, self.project.user / "minimal.yaml"):
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            music = next(module for module in data["modules"] if module["id"] == "rm_music")
            enabled = next(arg["enabled"] for arg in music["args"] if "enabled" in arg)
            self.assertIsInstance(enabled, bool)
            rendered = self.render(path)
            literal = "true" if enabled else "false"
            self.assertIn(f"static RmMusic rm_music(pwm_tim12_ch2, {literal}, 2048)", rendered)
            self.assertNotIn("static Mc02", rendered)
            self.assertNotIn("static BuzzerAlarm", rendered)

    def test_music_switch_generates_both_values(self):
        """临时配置独立覆盖true和false，不要求仓库默认产品必须开启音乐。"""
        data = yaml.safe_load(self.project.default_config.read_text(encoding="utf-8"))
        music = next(module for module in data["modules"] if module["id"] == "rm_music")
        switch = next(arg for arg in music["args"] if "enabled" in arg)
        with tempfile.TemporaryDirectory() as folder:
            for enabled in (True, False):
                literal = "true" if enabled else "false"
                config = Path(folder) / f"{literal}.yaml"
                switch["enabled"] = enabled
                config.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
                self.assertIn(f"pwm_tim12_ch2, {literal}, 2048)", self.render(config))

    def test_missing_hardware_rejected(self):
        """缺失硬件注册必须在生成时拒绝，而非留下失效依赖。"""
        original = self.project.default_config.read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "bad.yaml"
            config.write_text(original.replace("- pwm: pwm_tim12_ch2", "- pwm: missing_pwm"), encoding="utf-8")
            with self.assertRaises(ValueError):
                self.render(config)

    def test_describe_keeps_local_modules_separate_from_lock(self):
        """只读诊断识别仓内接口，且不把它们伪装成已锁外部仓库。"""
        result = describe(self.project)
        self.assertEqual(len(result["local_modules"]), 1)
        self.assertEqual(len(result["lock"]["modules"]), 2)
        self.assertEqual(len(result["modules"]), 3)
        errors = [d for d in result["diagnostics"] if d["severity"] == "error"]
        self.assertEqual(errors, [])

    def test_local_path_cannot_escape_bsp(self):
        """仓内模块的新路径登记不能把BSP外文件引入生成和构建。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry = root / "User_File/3_Modules/local_modules.yaml"
            registry.parent.mkdir(parents=True)
            registry.write_text("modules:\n- id: tasks/RmMusic\n  path: ../outside\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                local_records(root)

    def test_cli_propagates_generation_error(self):
        """生成器验证失败必须以非零退出，不能被项目包装器吞掉。"""
        original = self.project.default_config.read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "bad.yaml"
            config.write_text(original.replace("- pwm: pwm_tim12_ch2", "- pwm: missing_pwm"), encoding="utf-8")
            result = subprocess.run([sys.executable, str(ROOT / "tools/xrobot_project.py"),
                                     "-C", str(ROOT), "gen", "-c", str(config)],
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)

    def test_official_freshness_rejects_changed_input(self):
        """真实CMake检查通过新输入，并拒绝修改后未再生成的头文件。"""
        checker = ROOT / "Middlewares/Third_Party/LibXR/cmake/XRobotFreshness.cmake"
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            (base / "config.yaml").write_text("modules: []\n", encoding="utf-8")
            (base / "entry.cpp").write_text("XR_REGISTER(spi, LibXR::SPI);\n", encoding="utf-8")
            module = base / "sensor.hpp"
            module.write_text("class Sensor {};\n", encoding="utf-8")
            header = base / "xrobot_main.hpp"
            inputs = [("config", "config.yaml"), ("entry", "entry.cpp"), ("depends", "sensor.hpp")]
            content = "\n".join(f'// xrobot: {kind} "{path}"' for kind, path in inputs)
            content += f"\n// xrobot: digest {input_digest(header, inputs)}\n"
            header.write_text(content, encoding="utf-8")
            command = [os.environ.get("CMAKE_COMMAND", "cmake"),
                       f"-DXROBOT_MAIN_HEADER={header}", f"-DXROBOT_BSP_ROOT={base}", "-P", str(checker)]
            current = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(current.returncode, 0, current.stderr)
            module.write_text("class Sensor { int value; };\n", encoding="utf-8")
            stale = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(stale.returncode, 0)
            self.assertIn("stale", stale.stderr)


if __name__ == "__main__":
    unittest.main()
