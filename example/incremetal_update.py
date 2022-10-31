import numpy as np
from _voxbloxpy import EsdfMap, TsdfMap
import plotly.graph_objects as go


def create_esdf():
    quat_wxyz = [0.0, 1.0, 0.0, 0.0]
    pos = [-1.0, 0.0, 0.0]
    camera_pose = pos + quat_wxyz

    r_sphere = 0.5
    ylin = np.linspace(-1.0, 1.0, 100)
    zlin = np.linspace(-1.0, 1.0, 100)
    y_grid, z_grid = np.meshgrid(ylin, zlin)
    pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
    inside_circle = np.sum(pts_yz_plane ** 2, axis=1) < r_sphere - 0.02
    pts_yz_plane_inner = pts_yz_plane[inside_circle]
    pts_x = -np.sqrt(1.0 - np.sum(pts_yz_plane_inner ** 2, axis=1))

    pts = np.zeros((len(pts_x), 3))
    pts[:, 0] = pts_x
    pts[:, 1:] = pts_yz_plane_inner

    esdfmap = EsdfMap(0.01, 16)
    esdfmap.update(camera_pose, pts, False)
    return esdfmap

esdf = create_esdf()

xlin = np.linspace(-1.0, 1.0, 100)
ylin = np.linspace(-1.0, 1.0, 100)
zlin = np.linspace(-1.0, 1.0, 100)
X, Y, Z = np.meshgrid(xlin, ylin, zlin)
pts = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))
values = esdf.get_sd_batch(pts, 100.0)
print(values)

#fig = go.Figure(data=go.Isosurface(
#    x=X.flatten(),
#    y=Y.flatten(),
#    z=Z.flatten(),
#    value=values,
#    isomin=-1.0,
#    isomax=2.0,
#    opacity=0.1,
#    surface_count=10,
#    colorbar_nticks=5,
#    colorscale='Plotly3',
#    caps=dict(x_show=False, y_show=False)
#    ))
#fig.show()

#import matplotlib.pyplot as plt
#fig = plt.figure()
#ax = fig.add_subplot(projection='3d')
#ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
#plt.show()
