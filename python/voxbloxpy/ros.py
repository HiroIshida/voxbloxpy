import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import ros_numpy
import rospy
from sensor_msgs.msg import PointCloud2
from tf import TransformListener

from voxbloxpy import CameraPose, EsdfMap


@dataclass
class EsdfNodeConfig:
    point_cloud_topic: str
    world_frame: str = "map"
    voxel_size: float = 0.05
    voxel_per_side: int = 16


class EsdfNode:
    esdf: EsdfMap
    listener: TransformListener
    hook: Callable[[EsdfMap], None]
    config: EsdfNodeConfig
    callback_running: bool

    def __init__(self, config: EsdfNodeConfig, hook: Optional[Callable[[EsdfMap], None]] = None):
        self.esdf = EsdfMap.create(config.voxel_size, config.voxel_per_side)
        self.listener = TransformListener()
        rospy.Subscriber(config.point_cloud_topic, PointCloud2, self.point_cloud_cb)
        if hook is None:

            def hook(esdf: EsdfMap):
                pass

        self.hook = hook
        self.config = config
        self.callback_running = True

    def point_cloud_cb(self, pcloud: PointCloud2) -> None:
        if not self.callback_running:
            return
        target_frame = self.config.world_frame
        source_frame = pcloud.header.frame_id
        self.listener.waitForTransform(
            target_frame, source_frame, rospy.Time(), rospy.Duration(4.0)
        )
        pos, quat_xyzw = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        camera_pose = CameraPose(np.array(pos), quat_wxyz)

        pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcloud)

        # update esdf_map
        ts = time.time()
        self.esdf.update(camera_pose, pts)
        elapsed = time.time() - ts
        message = "esdf map is updated. elapsd time is {} sec".format(elapsed)
        rospy.loginfo(message)

        self.hook(self.esdf)
