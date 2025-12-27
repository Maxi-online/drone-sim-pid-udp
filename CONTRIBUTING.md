# Contributing

Thanks for your interest in contributing.

## Ground rules

- Keep changes focused and easy to review.
- Do not commit large binaries, datasets, or private/course assets.
- Do not commit secrets (API keys, tokens, credentials).

## Development setup

### 1) Create and activate a virtual environment

Windows (PowerShell):

```bash
python -m venv venv
.\venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

Linux/macOS:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip
```

### 2) Install dependencies

```bash
pip install -r requirements.txt
pip install -e .
```

## Code style

We use:

- **Black** for formatting
- **Ruff** for linting

Before opening a PR, run:

```bash
black .
ruff check . --fix
```

## How to test changes

Manual smoke checks:

- UDP demo:
  - `python scripts/visualizer.py`
  - `python scripts/plotter.py`
  - `python scripts/simulator.py`
- Integration demo:
  - `python quick_start_example.py`

## Pull requests

- Keep PRs small where possible.
- Include a short description: what changed and why.
- If you change behavior, describe how to verify it.

