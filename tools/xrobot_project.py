"""在自定义目录下运行官方XRobot CLI；参数与固定版本CLI一致。"""

from project_layout import configure_xrobot, write_local_cmake


def main():
    """适配物理目录后交给官方命令；输入CLI参数，输出官方验证/生成结果。"""
    configure_xrobot()
    from xrobot.cli import main as official_main, parser
    from xrobot.project import Project

    args = parser().parse_args()
    result = official_main()
    if result == 0 and args.verb in ("setup", "gen"):
        write_local_cmake(Project.discover(args.directory).root)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
