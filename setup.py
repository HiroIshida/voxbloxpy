import subprocess
import sys
from skbuild import setup

install_requires = [
    "numpy",
    "plotly",
]

setup(
    name="voxbloxpy",
    version="0.0.0",
    description="standalone voxblox python",
    author="Hirokazu Ishida",
    license="MIT",
    packages=["voxbloxpy"],
    package_dir={"": "python"},
    cmake_install_dir="python/voxbloxpy/",
    install_requires=install_requires,
)
