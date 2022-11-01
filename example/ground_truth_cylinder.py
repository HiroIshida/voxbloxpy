import numpy as np
import time
from _voxbloxpy import get_test_esdf
import plotly.graph_objects as go

esdf = get_test_esdf(0.05, 10, 640, 480)
print("finish creating esdf")

N = 50
xlin = np.linspace(-3.0, 3.0, N)
ylin = np.linspace(-3.0, 3.0, N)
zlin = np.linspace(-2.0, 4.0, N)
X, Y, Z = np.meshgrid(xlin, ylin, zlin)
pts_global = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))

ts = time.time()
values = np.array(esdf.get_sd_batch(pts_global, 100.0))
values[values>99.0] = np.nan

fig = go.Figure(data=go.Volume(
    x=X.flatten(), y=Y.flatten(), z=Z.flatten(),
    value=values,
    isomin=-0.5,
    isomax=2.0,
    opacity=0.05,
    surface_count=10,
    colorscale='jet'
    ))
fig.show()
