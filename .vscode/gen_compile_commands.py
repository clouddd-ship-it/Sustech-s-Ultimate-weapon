#!/usr/bin/env python3
"""为 clangd 生成 compile_commands.json（无需 bear / CMake）。

用法: python3 .vscode/gen_compile_commands.py
"""
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BUILD_DIR = os.path.join(ROOT, "build")


def opencv_cflags() -> list[str]:
    try:
        return subprocess.check_output(
            ["pkg-config", "--cflags", "opencv4"], text=True
        ).split()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ["-I/usr/include/opencv4"]


def main() -> int:
    if not shutil.which("pkg-config"):
        print("[warn] 未找到 pkg-config，使用默认 -I/usr/include/opencv4", file=sys.stderr)

    os.makedirs(BUILD_DIR, exist_ok=True)
    entries = []
    for name in sorted(os.listdir(ROOT)):
        if not name.endswith((".cpp", ".cc", ".cxx")):
            continue
        src = os.path.join(ROOT, name)
        entries.append(
            {
                "directory": ROOT,
                "file": src,
                "command": " ".join(
                    ["/usr/bin/g++", "-std=c++17", "-g", "-O0", "-Wall", "-Wextra"]
                    + opencv_cflags()
                    + ["-c", src]
                ),
                "output": os.path.join(BUILD_DIR, name + ".o"),
            }
        )

    out = os.path.join(ROOT, "compile_commands.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(entries, f, indent=2, ensure_ascii=False)
    print(f"[clangd] 已写入 {out}（{len(entries)} 个源文件）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
