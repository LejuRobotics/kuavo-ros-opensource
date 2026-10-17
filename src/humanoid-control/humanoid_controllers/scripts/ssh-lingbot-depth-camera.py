#!/usr/bin/env python3
"""Start the licensed LingBot depth ROS bridge on the upper computer."""

import sys
from pathlib import Path

from ssh_executor import SSHExecutor


def main():
    allowed = {"--minimal-post", "--gaussian", "--no-gaussian"}
    unknown = [arg for arg in sys.argv[1:] if arg not in allowed]
    if unknown:
        print(f"未知参数: {' '.join(unknown)}", file=sys.stderr)
        return 2
    if "--gaussian" in sys.argv[1:] and "--no-gaussian" in sys.argv[1:]:
        print("--gaussian 与 --no-gaussian 不能同时使用", file=sys.stderr)
        return 2
    use_gaussian = "--gaussian" in sys.argv[1:]
    command_name = (
        "launch_lingbot_depth_camera_compact_minimal"
        if "--minimal-post" in sys.argv[1:]
        else "launch_lingbot_depth_camera_compact_legacy"
    )
    config_path = Path(__file__).resolve().with_name("remote-config.json")
    executor = SSHExecutor(str(config_path))
    if not executor.connect():
        print("无法连接上位机 192.168.26.12", file=sys.stderr)
        return 2
    try:
        commands = executor.config.get("commands", {}).get(command_name)
        if not isinstance(commands, list):
            print(f"未找到预定义命令: {command_name}", file=sys.stderr)
            return 2
        gaussian_value = "true" if use_gaussian else "false"
        rendered_commands = [
            command.replace("__USE_GAUSSIAN__", gaussian_value)
            for command in commands
        ]
        results = executor.execute_commands(rendered_commands)
        for exit_code, output, error in results:
            if output:
                print(output)
            if error:
                print(error, file=sys.stderr)
            if exit_code != 0:
                return exit_code
        return 0
    finally:
        executor.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
