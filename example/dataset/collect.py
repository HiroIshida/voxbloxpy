import rospy
import numpy as np
import tf
import time
import ros_numpy
import pickle
from sensor_msgs.msg import PointCloud2
from voxbloxpy import CameraPose

def rotation_x(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])


def rotation_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def rotation_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

rospy.init_node("pointcloud collector")

class Collector:
    listener: tf.TransformListener
    finish: bool

    def __init__(self):
        self.listener = tf.TransformListener()
        rospy.Subscriber("/kinect_head/depth_registered/points", PointCloud2, self.callback)
        self.finish = False

    def callback(self, pcloud: PointCloud2):
        if self.finish:
            return
        target_frame = "base_link"
        source_frame = pcloud.header.frame_id

        self.listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
        pos, quat_xyzw = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        camera_pose = CameraPose(np.array(pos), quat_wxyz)

        pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcloud)

        #file_name = "pcloud_with_camera_pose-{}.pkl".format(time.strftime("%Y%m%d%H%M%S"))
        file_name = "pcloud_with_camera_pose.pkl"
        with open(file_name, "wb") as f:
            pickle.dump((pts, camera_pose), f)
        self.finish = True

col = Collector()
while not col.finish:
    time.sleep(2)
