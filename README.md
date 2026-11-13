# Quadcopter Dynamics, PID Control & UDP Telemetry

**A Python simulation toolkit for multicopter dynamics, cascaded control and telemetry-driven visualization.**

The project separates the vehicle model, controller, telemetry transport and visualization so they can be inspected and iterated independently. It is a control/simulation project; the core controller is classical PID rather than a trained policy.

## Engineering focus

- A multicopter dynamics model in [src/multicopter_model](src/multicopter_model).
- A cascaded position → velocity → attitude → rate controller and motor mixing.
- UDP telemetry connecting a simulator process with VisPy visualization/plotting tools.
- A PyBullet integration wrapper for single-process experiments.
- Separate optional DroneKit/MAVLink and AirSim examples with their own runtime requirements.

## Scope

The core UDP demo separates simulation from presentation. PyBullet provides an alternative single-process integration, while DroneKit/MAVLink and AirSim examples connect to separately configured runtimes. Telemetry payload variants and the setup for each path are documented below. Hardware flight qualification is outside the repository's simulation scope.

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

If PowerShell activation is restricted, run the environment's Python explicitly, such as `venv/Scripts/python.exe scripts/simulator.py`; changing the machine's execution policy is not required.

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

- the numeric `index` selecting a telemetry channel.

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

Platform note:
- **ROS2/Gazebo workflows are not a supported target for Windows** in this repo. If you
  need ROS2/Gazebo, use Linux (or WSL2 where appropriate) and keep that integration separate.

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

## Конфигурация, приватность и воспроизведение

Для локального UDP-сценария внешние AI-ключи не нужны. Запускайте примеры на loopback или в изолированной сети и используйте синтетическую телеметрию. Симуляция не подтверждает пригодность контроллера для реального полёта; условия эксперимента и настройки PID следует сохранять вместе с результатами.

Секреты храните в игнорируемых локальных файлах или переменных окружения. Безопасные примеры содержат только структуру настройки. Не передавайте секреты в командной строке и не добавляйте заполненные `.env`, приватные ключи, базы данных и логи в коммиты.

Workflow [Secret scan](.github/workflows/secret-scan.yml) проверяет текущие файлы и входящие коммиты Gitleaks 8.30.1 с редактированием значений в выводе. Это проверка на известные шаблоны, а не подтверждение отсутствия всех возможных утечек. Удаление значения из HEAD не очищает старую историю; порядок действий описан в [SECURITY.md](SECURITY.md).
