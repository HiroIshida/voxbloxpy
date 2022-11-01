## voxbloxpy [![CI](https://github.com/HiroIshida/voxbloxpy/actions/workflows/test.yaml/badge.svg)](https://github.com/HiroIshida/voxbloxpy/actions/workflows/test.yaml) [![pypi-version](https://badge.fury.io/py/voxbloxpy.svg)](https://pypi.org/project/voxbloxpy/)

This repository `voxbloxpy` provides the python-wrapper of the voxblox. [voxblox](https://github.com/ethz-asl/voxblox) is a ROS-based revolutionary project for online creation of signed distance field.

However, there are two unsatisfying things for me about the original repository:
- it depends on ROS package system (even voxblox core library)
- does not provide python interface

So, we copied the official voxblox source code to under `/voxblox` and edit the `CMakeLists.txt` and git submodules so that it can be build without `catkin`.
Then create a easy-to-use and easy-to-install python wrapper (see `wrapper.cpp` for the source code).

## installation

### Install from pypi
```bash
sudo apt-get install libgoogle-glog-dev
pip3 install voxbloxpy
```
Note: Dependency on `libgoogle-glog-dev` is kind of a pain, though it can be resolved by apt install. I'm planning to remove this dependency by building glog from source and build a static library. 

### Source build
```bash
git clone https://github.com/HiroIshida/voxbloxpy.git
cd voxbloxpy
git submodule update --init
sudo apt-get install libeigen3-dev libgtest-dev libgflags-dev libgoogle-glog-dev libprotobuf-dev
pip3 install -e .
```
