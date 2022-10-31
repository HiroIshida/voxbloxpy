import time
import numpy as np
import matplotlib.pyplot as plt
from _voxbloxpy import EsdfMap, TsdfMap
import plotly.graph_objects as go


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

    esdfmap = EsdfMap(0.05, 32)

    esdfmap.update(camera_pose, pts, True)
    n = esdfmap.get_num_alloc_block()
    print("allocated block: {}".format(n))

    return esdfmap

esdf = create_esdf(sphere=False, debug_view=False)
block_origins = esdf.get_block_origins()

xlin = np.linspace(-1.0, 1.0, 60)
ylin = np.linspace(-1.0, 1.0, 60)
zlin = np.linspace(-1.0, 1.0, 60)
X, Y, Z = np.meshgrid(xlin, ylin, zlin)
pts = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))

ts = time.time()
values = np.array(esdf.get_sd_batch(pts, 100.0))
values[values==100.0] = np.nan
print(time.time() - ts)

fig = go.Figure(data=go.Volume(
    x=X.flatten(), y=Y.flatten(), z=Z.flatten(),
    value=values,
    isomin=-0.1,
    isomax=1.0,
    opacity=0.05,
    surface_count=10,
    colorscale='jet'
    ))
fig.show()
