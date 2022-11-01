import pickle
import time
from pathlib import Path

import numpy as np

from voxbloxpy import CameraPose, EsdfMap, Grid

source_path = Path(__file__).resolve().parent.expanduser()
dataset_path = source_path / "dataset"

cloud_with_camera_list = []
for file_path in dataset_path.iterdir():
    if file_path.name.endswith(".pkl"):
        print("load pkl: {}".format(file_path))
        with file_path.open(mode="rb") as f:
            cloud_with_camera_list.append(pickle.load(f))

esdf = EsdfMap.create(0.02)
for cloud_with_camera in cloud_with_camera_list:
    pts, camera_pose = cloud_with_camera
    camera_pose: CameraPose
    ts = time.time()
    esdf.update(camera_pose, pts)
    print("elapsed time to update esdf {}".format(time.time() - ts))

grid = Grid(np.array([0.3, -1.0, 0.0]), np.array([2.0, 1.0, 2.0]), (100, 100, 100))
gridsdf = esdf.get_grid_sdf(grid)
gridsdf.render_volume(isomin=-0.1, isomax=0.2)
