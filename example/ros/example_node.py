import time
from pathlib import Path
from typing import List

import rospy

try:
    import kaleido  # noqa
except ImportError:
    message = "kaleido is required for plotly figure export. >> pip3 install kaleido"
    raise ImportError(message)

from voxbloxpy import EsdfMap, GridSDF
from voxbloxpy.ros import EsdfNode, EsdfNodeConfig

rospy.init_node("esdf_node")

topic = "/kinect_head/depth_registered/points"
world_frame = "/map"
config = EsdfNodeConfig(point_cloud_topic=topic, world_frame=world_frame)

grid_sdf_list: List[GridSDF] = []


def hook(esdf_map: EsdfMap):
    # this function might be too heavy.
    # I just do the following to make an animation. but do not do that
    # in the realtime application!
    ts = time.time()
    info = esdf_map.get_voxel_info()
    measure_grid = info.get_boundary_grid(grid_size=0.1)
    grid_sdf = esdf_map.get_grid_sdf(measure_grid)
    grid_sdf_list.append(grid_sdf)
    te = time.time()
    rospy.loginfo("elapsed time for getting gridsdf {} sec".format(te - ts))


node = EsdfNode(config, hook=hook)

time.sleep(20)  # stop node after a while
node.callback_running = False

# save figures as png
fig_path = Path(__file__).resolve().parent / "figs"
fig_path.mkdir(exist_ok=True)
for i, grid_sdf in enumerate(grid_sdf_list):
    fig = grid_sdf.render_volume(isomin=-0.2, isomax=2.0, show=False)
    name = fig_path / "sdfplot-{:03d}.png".format(i)
    rospy.loginfo("writing figure to {}".format(name))
    fig.write_image(name)

# save final figure as html
file_name = fig_path / "final_sdf.html"
fig = grid_sdf_list[-1].render_volume(isomin=-0.2, isomax=2.0, show=False)
fig.write_html(file_name)
