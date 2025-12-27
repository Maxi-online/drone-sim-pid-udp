# Drone Simulator (Python)

This repository contains a Python quadcopter simulator with:

- A multicopter dynamics model (`src/multicopter_model/`)
- A cascaded PID controller (position → velocity → attitude → rate)
- UDP telemetry streaming for real-time visualization and plotting (`scripts/`)
- Optional demos for DroneKit (SITL/MAVLink) and AirSim (separate setup)

## Requirements

- **Python**: 3.8+ (recommended)
- **OS**: Windows, Linux, macOS

Notes:
- The core simulator/visualizer uses **NumPy** and **VisPy**.
- The `SimulatorIntegration` wrapper requires **PyBullet**.
- DroneKit/AirSim demos are optional and have extra dependencies.

## Installation (recommended: virtual environment)

### Windows (PowerShell)

```bash
python -m venv venv
.\venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

If PowerShell blocks activation, run (once, as Administrator):

```bash
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Linux / macOS (bash/zsh)

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

## Quick start (UDP simulator + tools)

Run these in **separate terminals** (activate the same venv in each terminal).

### 1) Start the visualizer

```bash
python scripts/visualizer.py
```

### 2) Start the plotter

```bash
python scripts/plotter.py
```

To change which signal is plotted, edit `scripts/plotter.py` and set:

- `index = ...`

The plotter expects **extended telemetry** (20 doubles / 160 bytes).

### 3) Start the simulator (sends UDP telemetry)

```bash
python scripts/simulator.py
```

Default UDP target is `127.0.0.1:12346`.

## Package layout

- `src/multicopter_model/`
  - `model.py`: quadcopter dynamics and integration
  - `controller.py`: cascaded PID controller + mixer
  - `pid.py`: PID primitive
  - `simulator.py`: UDP streamer glue
- `scripts/`
  - `simulator.py`: main entry point for the UDP demo
  - `visualizer.py`: VisPy 3D visualization (UDP receiver)
  - `plotter.py`: VisPy plot window (UDP receiver)
  - `sendMessageExample.py`: simple UDP packet sender (debug)
  - `airsim_controller.py`: optional AirSim bridge

## Integration API (PyBullet)

For a “single process” integration (no UDP required), use `SimulatorIntegration`:

```bash
python quick_start_example.py
```

This uses:

- `professional_robot_simulator.py` (PyBullet environment + simple controller)
- `simulator_integration.py` (threaded wrapper + telemetry export)

## Optional: DroneKit demos (SITL / MAVLink)

Files:

- `professional_drone_simulator.py`
- `advanced_mission_controller.py`
- `drone_telemetry_analyzer.py`

These require additional dependencies and a MAVLink endpoint (e.g. SITL).
If the dependencies are missing, these scripts raise a clear error message.

## Optional: AirSim demo

AirSim is not installed via `requirements.txt` by default.
If you want to use it, install separately after NumPy:

```bash
pip install airsim
```

Licensing note:

- AirSim is a separate project with its own license.
- This repository does **not** include AirSim source code or binaries.
- If you use AirSim, keep your AirSim checkout and Unreal projects outside this repo
  (or ensure they are not committed).

Then see:

- `scripts/start_airsim.py`
- `scripts/airsim_controller.py`

### AirSim bridge: how it works

The AirSim bridge connects to a running AirSim Multirotor instance, feeds the
AirSim state into the local controller, applies the controller outputs back to
AirSim, and streams telemetry over UDP so you can use the existing visualizer
and plotter.

### AirSim bridge: run

1) Start your AirSim environment (UE project) so that the simulator is running.
You can use:

```bash
python scripts/start_airsim.py
```

2) Start the UDP receivers (optional but recommended):

```bash
python scripts/visualizer.py
python scripts/plotter.py
```

3) Run the bridge:

```bash
python scripts/airsim_controller.py
```

### AirSim bridge: telemetry format

The bridge sends the same 18-double payload format expected by `scripts/visualizer.py`.
Rotor omegas are not available from AirSim in this simple bridge, so indices 13..16
are sent as zeros (propeller animation may not spin).

## Optional: Gazebo

Gazebo assets are **not shipped** with this repository to avoid licensing ambiguity.
If you want a Gazebo workflow, keep Gazebo models/worlds in a separate folder/repo and
adapt the integration scripts accordingly.

## Development

This repo uses:

- **Black** for formatting
- **Ruff** for linting

Run locally:

```bash
black .
ruff check . --fix
```

Configuration lives in `pyproject.toml`.

## License

Licensed under **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.
See `LICENSE`.

