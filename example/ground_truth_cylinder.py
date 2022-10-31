import numpy as np
import time
from _voxbloxpy import get_test_esdf
import plotly
import plotly.graph_objects as go

esdf = get_test_esdf()
print("finish creating esdf")

xlin = np.linspace(-3.0, 3.0, 100)
ylin = np.linspace(-3.0, 3.0, 100)
zlin = np.linspace(-2.0, 4.0, 100)
X, Y, Z = np.meshgrid(xlin, ylin, zlin)
pts_local = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))

ts = time.time()
values = esdf.get_sd_batch(pts_local, 100.0)
print(values)


fig = go.Figure(data=go.Isosurface(
    x=X.flatten(),
    y=Y.flatten(),
    z=Z.flatten(),
    value=values,
    isomin=-1.0,
    isomax=2.0,
    opacity=0.1,
    surface_count=10,
    colorbar_nticks=5,
    colorscale='Plotly3',
    caps=dict(x_show=False, y_show=False)
    ))
fig.show()
