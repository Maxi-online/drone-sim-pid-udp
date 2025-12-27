#!/usr/bin/env python3

"""
Helper script to launch a local AirSim Unreal project.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

# Default settings for a multirotor
DEFAULT_SETTINGS = {
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ViewMode": "SpringArmChase",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "DefaultVehicleState": "Armed",
            "AutoCreate": True,
        }
    },
}


def ensure_settings():
    # AirSim reads settings.json from %USERPROFILE%\\Documents\\AirSim\\settings.json
    user_docs = Path.home() / "Documents"
    airsim_dir = user_docs / "AirSim"
    airsim_dir.mkdir(parents=True, exist_ok=True)
    settings_path = airsim_dir / "settings.json"
    if not settings_path.exists():
        with open(settings_path, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_SETTINGS, f, indent=2)
    return str(settings_path)


def start_airsim(airsim_exe_path: str):
    if not os.path.exists(airsim_exe_path):
        print(f"AirSim executable not found: {airsim_exe_path}")
        sys.exit(1)
    subprocess.Popen([airsim_exe_path], cwd=os.path.dirname(airsim_exe_path))
    print("AirSim started.")


if __name__ == "__main__":
    # Path to Blocks UE project in your AirSim folder
    project_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "AirSim",
        "Unreal",
        "Environments",
        "Blocks",
        "Blocks.uproject",
    )
    project_path = os.path.abspath(project_path)

    if not os.path.exists(project_path):
        print(f"Blocks.uproject not found: {project_path}")
        sys.exit(1)

    # Find UnrealEditor.exe for UE5
    possible_ue_paths = [
        f"C:\\Users\\{os.environ.get('USERNAME')}\\AppData\\Local\\UnrealEngine\\5.6\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
    ]

    ue_editor = None
    for path in possible_ue_paths:
        if os.path.exists(path):
            ue_editor = path
            break

    if not ue_editor:
        print("UnrealEditor.exe for UE5 was not found.")
        print("Check your Unreal Engine installation (5.4+).")
        sys.exit(1)

    settings_path = ensure_settings()
    print(f"Settings: {settings_path}")
    print(f"Launching UE project: {project_path}")

    # Launch UE project
    subprocess.Popen([ue_editor, project_path])
    print("Unreal Engine started with the Blocks project.")
