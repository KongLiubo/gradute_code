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

# RGB相机时的固定Zc
fixed_Zc = 475
# --------------------------------------------
# 转换矩阵T  [zu, zv, z, 1] 转化为[x, y, z, 1], 之后再处理为[x, y, z]
trans_matrix = np.array(([1.77690711e-04, -9.94581002e-07, -2.53595488e-01, -1.28067557e+02],
                         [-8.28352507e-07, -1.77416613e-04,  2.32549211e-01, -4.95282354e+02],
                         [-2.93210880e-06, -1.01511611e-05, -9.84328578e-01,  5.42181949e+02],
                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]), dtype=np.double)
# --------------------------------------------

if __name__ == "__main__":
    # 在该节点中打开图像采集程序
    print("Opening the image capture program ......")
    os.system("rosrun image_deal image_collect_once")

    # 初始化节点
    rospy.init_node("static_image_to_coordinate", anonymous=False)
    # 图片地址获取
    image_address = path + "/src/image_deal/image_transfer/image.jpg"
    # rospy.loginfo(image_address)
    # image_address = "/home/klb/catkin_ws/catkin_jaka/src/image_deal/image_transfer/image.jpg"

    # 创建处理对象
    goal_image = vision_localization()
    perations_num = 6   #开运算次数，过低捕捉不到，过高在理想边框中之内


    ###############################多物体抓取
    pixel_threshold = 2000
    
    # # 二值化
    count, pixel_coordinate = goal_image.obtain_xiangsu_coordinate_update(image_address, perations_num, pixel_threshold)

    # # canny算子
    # low_threshold = 50
    # high_threshold = 150
    # count, pixel_coordinate = goal_image.obtain_xiangsu_coordinate_canny(image_address, low_threshold, high_threshold, pixel_threshold)

    print(count)
    print(pixel_coordinate)

    pixel_coordinate = np.array(pixel_coordinate)

    # 进行初始化
    list_world_coordinate = [0, 0, 0, 0]
    for i in range(count):

        if count <= 1:
            center_x_demo = pixel_coordinate[0]
            center_y_demo = pixel_coordinate[1]
            distance_zc_demo = fixed_Zc     #注意！
            # center_Rz_demo = pixel_coordinate[2]
            # pixel_Rz ——> world_Rz
            center_Rz_array_pixel = np.asarray([0, 0, pixel_coordinate[2]])
            center_Rz_array_world = np.dot(trans_matrix[0:3, 0:3], center_Rz_array_pixel)
            center_Rz_demo = center_Rz_array_world[2]


        else:
            center_x_demo = pixel_coordinate[i][0]
            center_y_demo = pixel_coordinate[i][1]
            distance_zc_demo = fixed_Zc     #注意！
            # center_Rz_demo = pixel_coordinate[i][2]
            # pixel_Rz ——> world_Rz
            center_Rz_array_pixel = np.asarray([0, 0, pixel_coordinate[i][2]])
            center_Rz_array_world = np.dot(trans_matrix[0:3, 0:3], center_Rz_array_pixel)
            center_Rz_demo = center_Rz_array_world[2]
            
        print("[u, v , z, Rz] = ", center_x_demo, center_y_demo, distance_zc_demo, center_Rz_demo)
        xiangsu_coordinate = np.array([center_x_demo*distance_zc_demo, center_y_demo*distance_zc_demo, distance_zc_demo, 1])
        world_coordinate_demo = np.dot(trans_matrix, xiangsu_coordinate)
        world_coordinate = np.array([world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2], center_Rz_demo])
        print("world_coordinate = ", world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2] )
        print("rotate_angle_rz = ", center_Rz_demo)

        if i == 0:
            list_world_coordinate = [float(world_coordinate_demo[0]), float(world_coordinate_demo[1]), float(world_coordinate_demo[2]), float(center_Rz_demo)]
        else:
            list_world_coordinate = list_world_coordinate + [float(world_coordinate_demo[0]), float(world_coordinate_demo[1]), float(world_coordinate_demo[2]), float(center_Rz_demo)]

        # rospy.set_param("/world_coordinate_x_{}".format(i), float(world_coordinate_demo[0]))
        # rospy.set_param("/world_coordinate_y_{}".format(i), float(world_coordinate_demo[1]))
        # rospy.set_param("/world_coordinate_z_{}".format(i), float(world_coordinate_demo[2]))
        # rospy.set_param("/rotate_angle_rz_{}".format(i), float(center_Rz_demo))
    
    # list_world_coordinate.num = count * 4
    rospy.set_param("list_world_coordinate_pose", list_world_coordinate)
    rospy.set_param("goal_count", count)

    ###############################

    ###############################单一物体抓取
    # # 获取目标物体像素坐标（x，y）
    # center_x, center_y, rotate_angle = goal_image.obtain_xiangsu_coordinate(image_address, perations_num)  
    # # 相机与目标上表面距离zc 深度相机应该为输入形式，此处给定
    # distance_zc = fixed_Zc     #注意！
    # print("[u, v , z] = ", center_x, center_y, distance_zc)

    # # ---------转化为世界坐标--------------------
    # # 将上述参数换为[zu, zv, z, 1]
    # xiangsu_coordinate = np.array([center_x*distance_zc, center_y*distance_zc, distance_zc, 1])
    # world_coordinate_demo = np.dot(trans_matrix, xiangsu_coordinate)
    # # world_coordinate_demo[2] = 
    # world_coordinate = np.array([world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2], rotate_angle])
    # print("world_coordinate = ", world_coordinate_demo[0], world_coordinate_demo[1], world_coordinate_demo[2] )
    # print("rotate_angle_rz = ", rotate_angle)

    # # 将位姿信息发送到参数服务器中
    # rospy.set_param("/world_coordinate_x", float(world_coordinate[0]))
    # rospy.set_param("/world_coordinate_y", float(world_coordinate[1]))
    # rospy.set_param("/world_coordinate_z", float(world_coordinate[2]))
    # rospy.set_param("/rotate_angle_rz", float(rotate_angle))

    print("operation has successed !")

    # 之后可以把该wait注释
    cv2.waitKey(0)
    

    


