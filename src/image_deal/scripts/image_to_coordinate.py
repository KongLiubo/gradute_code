#! /usr/bin/env python

#-----------------------
# 获取子文件路径 
import os
import sys
path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/image_deal/scripts")
from image_class import vision_localization
#-----------------------

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

bridge = CvBridge()

# RGB相机时的固定Zc
fixed_Zc = 475
# --------------------------------------------
# 转换矩阵T  [zu, zv, z, 1] 转化为[x, y, z, 1], 之后再处理为[x, y, z]
trans_matrix = np.array(([1.76334702e-04,  -7.95250888e-06, -3.51982445e-01, -7.25197245e+01],
                         [-5.60053504e-06, -1.76804538e-04,  2.90550122e-01, -5.18367016e+02],
                         [-2.29135197e-05, -1.79851583e-05, -9.37934215e-01,  5.02098441e+02],
                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]), dtype=np.double)
# --------------------------------------------

def Img_callback(data):
    try:
        # 创建处理对象
        goal_image = vision_localization()
        # 将sensor_msgs/Image类型的消息转化为opencv可处理的消息
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        perations_num = 6   #开运算次数，过低捕捉不到，过高在理想边框中之内
        # 获取目标物体像素坐标（x，y）
        center_x, center_y, rotate_angle = goal_image.obtain_xiangsu_coordinate_plus(cv_image, perations_num)   
        # 相机与目标上表面距离zc 深度相机应该为输入形式，此处给定
        distance_zc = fixed_Zc     #注意！
        print("[u, v , z] = ", center_x, center_y, distance_zc)


        # ---------转化为世界坐标--------------------
        # 将上述参数换为[zu, zv, z, 1]
        xiangsu_coordinate = np.array([center_x*distance_zc, center_y*distance_zc, distance_zc, 1])
        world_coordinate_demo = np.dot(trans_matrix, xiangsu_coordinate)
        # world_coordinate_demo[2] = 
        world_coordinate = np.array([world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2], rotate_angle])
        print("world_coordinate = ", world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2] )
        print("rotate_angle_rz = ", rotate_angle)

        # -----------------------------------------
        # 将目标物快坐标以话题形式发布出去
        # 前三个数据为位置信息，最后一个数据为目标物体的rz姿态信息
        data = Float32MultiArray(data = world_coordinate)
        coordinate_pub.publish(data)

        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)


if __name__ == "__main__":
    # 初始化节点
    rospy.init_node("image_to_coordinate", anonymous=False)
    # os.system("gnome-terminal -- bash -c 'source ~/driver_ros/camera_driver_ros/ws_hk_mvs_ros/devel/setup.bash; roslaunch hk_camera hk_camera.launch'&")
    # os.system("gnome-terminal -- bash -c 'source ~/Jaka_ROS/jaka_ros_driver_cpp/devel/setup.bash; roslaunch jaka_ros_driver start.launch'&")

    # 订阅图像话题
    image_sub = rospy.Subscriber("/hk_camera_node/image_raw", Image, Img_callback, queue_size=100)
    coordinate_pub = rospy.Publisher("/goal_world_coordinate",Float32MultiArray,queue_size=10)

    # 反复spin
    rospy.spin()
