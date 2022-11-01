import time
import numpy as np
import matplotlib.pyplot as plt
from _voxbloxpy import EsdfMap, TsdfMap


def create_esdf(sphere: bool = True, debug_view: bool = True):
    quat_wxyz = [0.0, 1.0, 0.0, 0.0]
    pos = [-1.0, 0.0, 0.0]
    camera_pose = pos + quat_wxyz

    r_sphere = 0.5
    ylin = np.linspace(-1.0, 1.0, 100)
    zlin = np.linspace(-1.0, 1.0, 100)
    y_grid, z_grid = np.meshgrid(ylin, zlin)

    if sphere:
        pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
        inside_circle = np.sum(pts_yz_plane ** 2, axis=1) < r_sphere - 0.02
        pts_yz_plane_inner = pts_yz_plane[inside_circle]
        pts_x = -np.sqrt(1.0 - np.sum(pts_yz_plane_inner ** 2, axis=1)) + 1.0  # 1.0 for camera

        pts = np.zeros((len(pts_x), 3))
        pts[:, 0] = pts_x
        pts[:, 1:] = pts_yz_plane_inner
    else:
        pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
        pts_x = np.zeros(len(pts_yz_plane))
        pts = np.zeros((len(pts_x), 3))
        pts[:, 0] = pts_x
        pts[:, 1:] = pts_yz_plane

    if debug_view:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
        plt.show()

    esdfmap = EsdfMap(0.02, 32)

    esdfmap.update(camera_pose, pts, True)
    n = esdfmap.get_num_alloc_block()
    print("allocated block: {}".format(n))

    return esdfmap

esdf = create_esdf(sphere=True, debug_view=False)
block_origins = esdf.get_block_origins()

pts, dists, observed = esdf.get_voxel_info(True);
dists = np.array(dists)
print(len(pts))
pts = np.array(pts)

pts1 = pts[np.logical_and(np.abs(dists) < 0.02, observed)]
pts2 = pts[np.logical_and(np.abs(dists - 0.1) < 0.02, observed)]
pts3 = pts[np.logical_and(np.abs(dists - 0.2) < 0.02, observed)]
pts4 = pts[np.logical_and(np.abs(dists - 0.3) < 0.02, observed)]

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pts1[:, 0], pts1[:, 1], pts1[:, 2], c="yellow")
ax.scatter(pts2[:, 0], pts2[:, 1], pts2[:, 2], c="red")
ax.scatter(pts3[:, 0], pts3[:, 1], pts3[:, 2], c="blue")
ax.scatter(pts4[:, 0], pts4[:, 1], pts4[:, 2], c="green")
plt.show()
