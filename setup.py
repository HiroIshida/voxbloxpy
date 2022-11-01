import sys

# see 
# https://github.com/scikit-build/scikit-build-sample-projects/blob/master/projects/hello-pybind11/setup.py
try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="voxbloxpy",
    version="0.0.0",
    description="standalone voxblox python",
    author='Hirokazu Ishida',
    license="MIT",
    packages=["voxbloxpy"],
    package_dir={'': 'python'},
    cmake_install_dir='python/voxbloxpy/'
    )
