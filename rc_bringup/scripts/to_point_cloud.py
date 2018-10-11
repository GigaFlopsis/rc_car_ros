#! /usr/bin/env python
# coding: utf-8

import rospy
import pcl_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy

def on_new_point_cloud(data):
    pc = numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))
    seg = p.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    indices, model = seg.segment()

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/laserPointCLoud", PointCloud2, on_new_point_cloud)
rospy.spin()
