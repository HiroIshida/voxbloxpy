import rospy

from voxbloxpy.ros import EsdfNode, EsdfNodeConfig

rospy.init_node("esdf_node")

topic = "/kinect_head/depth_registered/points"
world_frame = "/map"
config = EsdfNodeConfig(point_cloud_topic=topic, world_frame=world_frame)
node = EsdfNode(config, hook=None)
rospy.spin()
