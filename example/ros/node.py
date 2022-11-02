from typing import List

import rospy

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
    grid_sdf.render_volume(isomin=-0.2, isomax=0.3)
    grid_sdf_list.append(grid_sdf)


node = EsdfNode(config, hook=hook)
rospy.spin()
