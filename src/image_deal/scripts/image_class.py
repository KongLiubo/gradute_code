#! /usr/bin/env python

import cv2
import numpy as np

import math

# 声明必要参数
PI = 3.1415926
distance_zc = 627.7   #该参数为 摄像头与目标物体的（Z轴）距离


class vision_localization:
    # 一些子函数

    # 计算旋转角rz
    def angle_rotate_offset(self, point_one, point_two, point_three):
        vector_one_to_two = point_one-point_two
        length_one_to_two = np.linalg.norm(vector_one_to_two)
        # print(length_one_to_two)

        vector_one_to_three = point_one-point_three
        length_one_to_three = np.linalg.norm(vector_one_to_three)
        # print(length_one_to_three)

        goal_point_one = point_one
        if length_one_to_two >= length_one_to_three:
            goal_point_two = point_two
        else:
            goal_point_two = point_three

        if goal_point_one[1] > goal_point_two[1]:
            jiaodu = np.arctan2((goal_point_one[1] - goal_point_two[1]), (goal_point_one[0] - goal_point_two[0]))
        else:
            jiaodu = np.arctan2((goal_point_two[1] - goal_point_one[1]), (goal_point_two[0] - goal_point_one[0]))

        rotate_angle = 90 - jiaodu * 180 / math.pi
        return rotate_angle

    # 输入像素坐标，获取世界坐标
    def calculate_world_coordinate(self, camera_matrix, Out_matrix, xiangsu_coordinate):
        demo_xiangsu = np.hstack([xiangsu_coordinate, [1]])
        print("xiangsu_coordinate = ", demo_xiangsu)
        # 在已知外参矩阵Out_matrix和内参矩阵camera_matrix，求取 像素坐标系->基坐标系的变换矩阵
        xiangshu_to_cam = np.linalg.inv(camera_matrix) * distance_zc
        tem_cam = np.dot(xiangshu_to_cam, demo_xiangsu)
        # 对摄像头坐标进行变换，由1*3矩阵变为1*4，由于使用dot，会自动转置
        demo_one = np.array([1])
        tem_cam = np.hstack([tem_cam, demo_one])
        # 对外参矩阵进行变换，形成4*4变换算子，以求逆
        demo_four = np.array([0, 0, 0, 1])
        trans_Out_matrix = np.vstack([Out_matrix, demo_four])
        cam_to_world = np.linalg.inv(trans_Out_matrix)
        demo_world_coordinate = np.dot(cam_to_world, tem_cam)
        world_coordinate = np.array([demo_world_coordinate[0], demo_world_coordinate[1], demo_world_coordinate[2]])
        return world_coordinate
    # -----------------------------------------------------------
    # 读取图片
    def ReadImg(self, image_address):
        # cv2.namedWindow('src', 0)
        # cv2.resizeWindow('src', 300, 300)
        img = cv2.imread(image_address, 1)
        # cv2.imshow('src', img)
        return img


    # 高斯滤波
    def GausBlur(self, src):
        # cv2.namedWindow('GausBlur', 0)
        # cv2.resizeWindow('GausBlur', 300, 300)
        dst = cv2.GaussianBlur(src, (5, 5), 1.5)
        # cv2.imshow('GausBlur', dst)
        return dst


    # 灰度处理
    def Gray_img(self, src):
        # cv2.namedWindow('gray', 0)
        # cv2.resizeWindow('gray', 300, 300)
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('gray', gray)
        return gray


    # 二值化
    def threshold_img(self, src):
        # cv2.namedWindow('threshold', 0)
        # cv2.resizeWindow('threshold', 300, 300)
           # 大津法
        ret, temp_binary = cv2.threshold(src, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        set_value_temp = 40
        set_value = ret - set_value_temp
        if set_value < 10:
            set_value = 10
        # set_value = 10
        # 固定阈值二值化
        ret, binary = cv2.threshold(src, set_value, 255, cv2.THRESH_BINARY)
        print("threshold value %s" % ret)
        # cv2.imshow('threshold', binary)
        return binary


    # 开运算操作
    def open_mor(self, src, perations_num):
        # cv2.namedWindow('open', 0)
        # cv2.resizeWindow('open', 300, 300)
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(src, cv2.MORPH_OPEN, kernel, iterations=perations_num)  # iterations进行3次操作
        # cv2.imshow('open', opening)
        return opening
    # 轮廓拟合
    def draw_shape(self, open_img, src):
        contours, hierarchy = cv2.findContours(open_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]  # 得到第一个的轮廓

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int_(box)
        cv2.drawContours(src, [box], 0, (0, 0, 255), 3)  # 画矩形框

        edge_point_one = box[0]
        edge_point_two = box[1]
        edge_point_three = box[2]
        edge_point_four = box[3]
        rotate_angle = self.angle_rotate_offset(edge_point_one, edge_point_two, edge_point_four)

        # 图像轮廓及中心点坐标
        M = cv2.moments(cnt)  # 计算第一条轮廓的各阶矩,字典形式
        center_x = int(M['m10'] / M['m00'])
        center_y = int(M['m01'] / M['m00'])
        # print('center_x:', center_x)
        # print('center_y:', center_y)
        cv2.circle(src, (center_x, center_y), 7, 128, -1)  # 绘制中心点
        str1 = '(' + str(center_x) + ',' + str(center_y) + ')'  # 把坐标转化为字符串
        cv2.putText(src, str1, (center_x - 50, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2,
                    cv2.LINE_AA)  # 绘制坐标点位

        cv2.namedWindow('show', 0)
        cv2.resizeWindow('show', 500, 500)
        cv2.imshow('show', src)
        return center_x, center_y, rotate_angle
    
    def canny_extraction_edge(self, src, low_threshold, high_threshold):
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)  # 转为灰度图
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        edge = cv2.Canny(blurred, low_threshold, high_threshold)  # 用Canny算子提取边缘

        # cv2.namedWindow('edge', 0)
        # cv2.resizeWindow('edge', 300, 300)
        # cv2.imshow('edge', edge)

        return edge

    # 轮廓拟合(升级版)
    def draw_shape_update(self, open_img, src, pixel_threshold):

        count = 0  # 轮廓个数

        contour = src.copy()
        (cnts, _) = cv2.findContours(open_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 轮廓检测
        cv2.drawContours(contour, cnts, -1, (0, 255, 0), 2)  # 绘制轮廓

        # cv2.namedWindow('contour', 0)
        # cv2.resizeWindow('contour', 300, 300)
        # cv2.imshow('contour', contour)

        for i, contour in enumerate(cnts):
            area = cv2.contourArea(contour)
            if area < pixel_threshold:  # 过滤面积小于15的形状
                continue
            count += 1

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int_(box)
            cv2.drawContours(src, [box], 0, (0, 0, 255), 2)  # 画矩形框

            # -------------------------
            edge_point_one = box[0]
            edge_point_two = box[1]
            edge_point_three = box[2]
            edge_point_four = box[3]

            rotate_angle = self.angle_rotate_offset(edge_point_one, edge_point_two, edge_point_four)
            # print(rotate_angle)
            # -------------------------

            # 图像轮廓及中心点坐标
            M = cv2.moments(contour)  # 计算第一条轮廓的各阶矩,字典形式
            center_x = int(M['m10'] / M['m00'])
            center_y = int(M['m01'] / M['m00'])
            # print('center_x:', center_x)
            # print('center_y:', center_y)

            cv2.circle(src, (center_x, center_y), 10, 128, -1)  # 绘制中心点
            str1 = '(' + str(center_x) + ',' + str(center_y) + ','+str(round(rotate_angle,1))+')'  # 把坐标转化为字符串
            cv2.putText(src, str1, (center_x - 50, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 0), 2,
                        cv2.LINE_AA)  # 绘制坐标点位
            # cv2.putText(src, str1, (box[0] - 50, box[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 0), 2,
            #             cv2.LINE_AA)  # 绘制坐标点位
            if count == 1:
                pixel_coordinate = np.array([center_x, center_y, rotate_angle], dtype=np.double)
            else:
                pixel_coordinate = np.vstack([pixel_coordinate, [center_x, center_y, rotate_angle]])

        cv2.namedWindow('show', 0)
        cv2.resizeWindow('show', 300, 300)
        cv2.imshow('show', src)

        return count, pixel_coordinate

    def obtain_xiangsu_coordinate(self, image_address, open_operations):
        src = self.ReadImg(image_address)
        gaus_img = self.GausBlur(src)
        gray_img = self.Gray_img(gaus_img)
        thres_img = self.threshold_img(gray_img)
        open_img = self.open_mor(thres_img, open_operations)
        center_x, center_y, rotate_angle = self.draw_shape(open_img, src)
        # cv2.waitKey(0)
        return center_x, center_y, rotate_angle
    def obtain_xiangsu_coordinate_plus(self, src, open_operations):
        gaus_img = self.GausBlur(src)
        gray_img = self.Gray_img(gaus_img)
        thres_img = self.threshold_img(gray_img)
        open_img = self.open_mor(thres_img, open_operations)
        center_x, center_y, rotate_angle = self.draw_shape(open_img, src)  
        return center_x, center_y, rotate_angle
    
    def obtain_xiangsu_coordinate_update(self, image_address, open_operations, pixel_threshold):
        src = self.ReadImg(image_address)
        gaus_img = self.GausBlur(src)
        gray_img = self.Gray_img(gaus_img)
        thres_img = self.threshold_img(gray_img)
        open_img = self.open_mor(thres_img, open_operations)
        count, pixel_coordinate = self.draw_shape_update(open_img, src, pixel_threshold)
        # cv2.waitKey(0)
        return count, pixel_coordinate
    
    def obtain_xiangsu_coordinate_canny(self, image_address, low_threshold, high_threshold, pixel_threshold):
        image = cv2.imread(image_address, 1)
        edge = self.canny_extraction_edge(image, low_threshold, high_threshold)
        count, pixel_coordinate = self.draw_shape_update(edge, image, pixel_threshold)
        # cv2.waitKey(0)
        return count, pixel_coordinate
    # -----------------------------------------------------------

