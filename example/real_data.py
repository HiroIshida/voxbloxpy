import pickle
import numpy as np
from pathlib import Path
from voxbloxpy import CameraPose, EsdfMap, Grid

source_path = Path(__file__).resolve().parent.expanduser()
dataset_path = source_path / "dataset"

cloud_with_camera_list = []
for file_path in dataset_path.iterdir():
    if file_path.name.endswith(".pkl"):
        with file_path.open(mode = "rb") as f:
            cloud_with_camera_list.append(pickle.load(f))

esdf = EsdfMap.create(0.02)
for cloud_with_camera in cloud_with_camera_list:
    pts, camera_pose = cloud_with_camera
    camera_pose: CameraPose
    #camera_pose.position += np.array([4.9, 0.0, 0.0])
    print(camera_pose)
    esdf.update(camera_pose, pts)

#info = esdf.get_voxel_info()
#info = info.filter(-0.1, 0.3)
#pts = info.origins
#import matplotlib.pyplot as plt
#fig = plt.figure()
#ax = fig.add_subplot(projection="3d")
#ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
#ax.set_xlabel("x")
#ax.set_ylabel("y")
#ax.set_zlabel("z")
#plt.show()

grid = Grid(np.array([0.3, -1.0, 0.0]), np.array([2.0, 1.0, 2.0]), (100, 100, 100))
gridsdf = esdf.get_grid_sdf(grid)
print(gridsdf.values)
gridsdf.render_volume()