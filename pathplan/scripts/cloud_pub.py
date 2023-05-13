#! /usr/bin/env python3.8
# encoding: utf-8
import open3d as o3d
import copy
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

# 类
class PointRead(object):
    def __init__(self):
        self.pcloud = rospy.Publisher('/read_pointcloud', PointCloud2, queue_size=1)
        data = o3d.io.read_point_cloud("./src/data/point_cloud/pointcloud_1.pcd")
        zpath = np.array(data.points)
        points_pcd = o3d.geometry.PointCloud()# 传入3d点云
        points_pcd.points = o3d.utility.Vector3dVector(zpath)
        # publish
        header = Header()
        header.frame_id = "map" 
        header.stamp = rospy.Time.now()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
        ]
        rate=rospy.Rate(1)
        pc = point_cloud2.create_cloud(header, fields, points_pcd.points)  
        while not rospy.is_shutdown():      
            self.pcloud.publish(pc)
            rate.sleep()

# 主函数
if __name__ == "__main__": 
    rospy.init_node("select_pc")
    # 发布点云话题
    PointRead()
    rospy.spin()