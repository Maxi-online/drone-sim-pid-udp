#!/usr/bin/env python3
"""
Helper script to build a local AirSim Unreal project with UE5.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

Important licensing note:
- AirSim is a separate project with its own license.
- This repository does NOT ship AirSim source code or binaries.
- This script is only a convenience wrapper for users who already have a local
  AirSim checkout and Unreal Engine installed.
"""
import os
import subprocess
import sys


def find_unreal_build_tool():
    """Find UnrealBuildTool.exe for UE5."""
    possible_paths = [
        "D:\\games\\UE_5.6\\Engine\\Binaries\\DotNET\\UnrealBuildTool\\UnrealBuildTool.exe",
        f"C:\\Users\\{os.environ.get('USERNAME')}\\AppData\\Local\\UnrealEngine\\5.6\\Engine\\Binaries\\DotNET\\UnrealBuildTool\\UnrealBuildTool.exe",
        "C:\\Program Files\\Epic Games\\UE_5.6\\Engine\\Binaries\\DotNET\\UnrealBuildTool\\UnrealBuildTool.exe",
    ]

    for path in possible_paths:
        if os.path.exists(path):
            return path

    print("UnrealBuildTool.exe not found")
    return None


def find_unreal_editor():
    """Find UnrealEditor.exe."""
    possible_paths = [
        f"C:\\Users\\{os.environ.get('USERNAME')}\\AppData\\Local\\UnrealEngine\\5.6\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
        "D:\\games\\UE_5.6\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
        "C:\\Program Files\\Epic Games\\UE_5.6\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
    ]

    for path in possible_paths:
        if os.path.exists(path):
            return path

    print("UnrealEditor.exe not found")
    return None


def rebuild_project():
    """Build the local AirSim Unreal project (user-provided checkout)."""

    # Project paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    project_path = os.path.join(
        project_root, "AirSim", "Unreal", "Environments", "Blocks", "Blocks.uproject"
    )

    if not os.path.exists(project_path):
        print(f"Project not found: {project_path}")
        return False

    # Find UE5 tools
    unreal_editor = find_unreal_editor()
    if not unreal_editor:
        return False

    print(f"UnrealEditor: {unreal_editor}")
    print(f"Project: {project_path}")

    # Find UnrealBuildTool for compilation
    build_tool = find_unreal_build_tool()
    if not build_tool:
        print("UnrealBuildTool not found!")
        return False

    print(f"UnrealBuildTool: {build_tool}")

    # Generate Visual Studio project files
    print("Generating project files...")
    try:
        cmd = [unreal_editor, "-projectfiles", project_path]
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        print("Project files generated.")
    except Exception as e:
        print(f"Warning during project file generation: {e}")

    # Build via UnrealBuildTool
    print("Building project via UnrealBuildTool...")
    try:
        cmd = [
            build_tool,
            "Blocks",
            "Development",
            "Win64",
            f"-project={project_path}",
            "-WaitMutex",
            "-FromMsBuild",
        ]
        print(f"Build command: {' '.join(cmd)}")
        result = subprocess.run(
            cmd, check=True, capture_output=True, text=True, timeout=600
        )
        print("Project built successfully.")
        print("Stdout:", result.stdout[-500:])  # last 500 characters
        return True
    except subprocess.TimeoutExpired:
        print("Build timed out (10 minutes)")
        return False
    except subprocess.CalledProcessError as e:
        print(f"Build error: {e}")
        print(f"Stdout: {e.stdout[-1000:]}")  # last 1000 characters
        print(f"Stderr: {e.stderr[-1000:]}")
        return False


if __name__ == "__main__":
    print("Building local AirSim Unreal project for UE5...")
    success = rebuild_project()

    if success:
        print("Rebuild completed successfully.")
        print("You can now start the project via start_airsim.py")
    else:
        print("Rebuild failed!")
        sys.exit(1)
