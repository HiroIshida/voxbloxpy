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
    info = esdf_map.get_voxel_info()
    measure_grid = info.get_boundary_grid(grid_size=0.1)
    grid_sdf = esdf_map.get_grid_sdf(measure_grid)
    grid_sdf_list.append(grid_sdf)


node = EsdfNode(config, hook=hook)

time.sleep(5)
node.callback_running = False

# save figures
fig_path = Path(__file__).resolve().parent / "figs"
fig_path.mkdir(exist_ok=True)
for i, grid_sdf in enumerate(grid_sdf_list):
    fig = grid_sdf.render_volume(isomin=-0.2, isomax=0.3, show=False)
    name = fig_path / "sdfplot-{}.png".format(i)
    rospy.loginfo("writing figure to {}".format(name))
    fig.write_image(name)
