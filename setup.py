"""
Package definition for `multicopter_model`.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

from setuptools import find_packages, setup

setup(
    name="multicopter_model",
    version="1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
