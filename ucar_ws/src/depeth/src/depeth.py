#!/usr/bin/env python
# coding=UTF-8
import rospy,math,random
import numpy as np
import message_filters

from sensor_msgs.msg import PointCloud2,Image
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

def my_callback(sub_scan, sub_image,sub_pose):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    time.sleep(1)
    print type(gen)
    for p in gen:
      print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])



if __name__ == '__main__':
    rospy.init_node('listener',anonymous=True)

    sub_scan = message_filters.Subscriber('/scan', PointCloud2, queue_size=1)
    sub_image=message_filters.Subscriber('/cam_image', Image, queue_size=1)
    sub_pose  = message_filters.Subscriber('car_pose', PoseWithCovarianceStamped, queue_size=1)
    
    sync = message_filters.ApproximateTimeSynchronizer([sub_scan, sub_image,sub_pose],10,0.1,allow_headerless=True)

    sync.registerCallback(my_callback)

    rospy.spin()
