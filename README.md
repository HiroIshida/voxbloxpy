# voxbloxpy [![CI](https://github.com/HiroIshida/voxbloxpy/actions/workflows/test.yaml/badge.svg)](https://github.com/HiroIshida/voxbloxpy/actions/workflows/test.yaml) [![pypi-version](https://badge.fury.io/py/voxbloxpy.svg)](https://pypi.org/project/voxbloxpy/)

[voxblox](https://github.com/ethz-asl/voxblox) is a ROS-based revolutionary project for online creation of signed distance field. This repository `voxbloxpy` provides the python-wrapper of the [voxblox](https://github.com/ethz-asl/voxblox) and some utils for volumetric rendering using `plotly`. This python package is **standalone**, that is, the package is ros-dependencies-free and can be installed from pypi.

The following animation is created using point cloud collected by a robot rotating in our lab.

https://user-images.githubusercontent.com/38597814/199616755-02bcaed4-1170-4d88-87d2-995a67663d41.mp4

This project is just a wrapper. So, please cite the paper (Oleynikova+, IROS 2017) for the original project when you use this in your research.
```latex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```



# installation
**NOTE: ROS not required**

## Install from pypi
```bash
sudo apt-get install libgoogle-glog-dev
pip3 install voxbloxpy
```
Note: Dependency on `libgoogle-glog-dev` is kind of a pain, though it can be resolved by apt install. I'm planning to remove this dependency by building glog from source and build a static library. 

## Source build
```bash
git clone https://github.com/HiroIshida/voxbloxpy.git
cd voxbloxpy
git submodule update --init
sudo apt-get install libeigen3-dev libgtest-dev libgflags-dev libgoogle-glog-dev libprotobuf-dev
pip3 install -e .
```

# Run demo (real dataset)
download dataset (pickled pointcloud and camera poses)
```bash
pip3 install gdown  # if not installed yet
cd example/dataset/
./download_dataset.sh
```
The dataset is created by the scan when PR2 robot is directing toward the fridge with the opened door.


Then run esdf creation demo
```bash
python3 example/real_data.py --visualize
```
The bottom left figure shows the rviz-image at the scan time, and the bottom right figure shows the output of the visualization.

<img src='https://user-images.githubusercontent.com/38597814/199342789-19f91722-3880-417d-b873-e0b735049496.png' width=30%> <img src='https://user-images.githubusercontent.com/38597814/199342783-a4dd2a50-ee56-46e6-ace2-8dcd48d748be.png' width=50%>

# Run demo playing rosbag (you need to have ROS)
```
cd example/ros
bash download_rosbag.sh
roscore  # run in different terminal
rosbag play pr2_jsk_73b2_movearound.bag # run in different terminal
python3 example_node.py
```
The sequence of figures and interactive html will be dumped in `/example/ros/figs`.
See the mp4 video at the top of this README for the visualization of the result.
