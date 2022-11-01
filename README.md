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

### Run demo (real dataset)
download dataset (pickled pointcloud and camera poses)
```bash
pip3 install gdown  # if not installed yet
cd example/dataset/
./download_dataset.sh
```

The dataset is created by the scan when PR2 robot is directing toward the fridge with the opened door. The rviz image is at the scan time is the following.

<img src='https://user-images.githubusercontent.com/38597814/199342789-19f91722-3880-417d-b873-e0b735049496.png' width=30%>

Then run esdf creation demo
```bash
python3 example/real_data.py --visualize
```
The output figure is like

<img src='https://user-images.githubusercontent.com/38597814/199342783-a4dd2a50-ee56-46e6-ace2-8dcd48d748be.png' width=50%>
