#!/usr/bin/python
#-----------------------
# 获取子文件路径 
import os
import sys
path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/image_deal/scripts")
#-----------------------

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


def callback(imgmsg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    cv2.imshow("listener", img)
    cv2.waitKey(3)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hk_camera_node/image_raw", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()

