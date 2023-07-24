 
#include <ros/ros.h>
 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
 
#include "std_msgs/Int32.h"
#include <vector>
#include <std_msgs/String.h>
#include <stdio.h>
 
#include <sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include "ros/package.h"
 
using namespace cv;
using namespace std;
 
//  使用waitForMessage直接获取msg数据（而非callback）
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_collect_once");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1.0);
    ROS_INFO_STREAM("Preparing for collecting the image!");

    boost::shared_ptr<sensor_msgs::Image const> image_sub;  
    image_sub = ros::topic::waitForMessage<sensor_msgs::Image>("/hk_camera_node/image_raw",ros::Duration(3));
    //image_sub = ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw",(ros::NodeHandle *)&nh, ros::Duration(3));
    if(image_sub != NULL)
    {
        sensor_msgs::Image  image_raw = *image_sub;
        //cout<<"battery:"<<  bat  <<endl;
        //cout<<"battery:"<<  image_sub  <<endl;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
	    Mat img = cv_ptr -> image;
        std::string image_name = "image.jpg";

        // 相对路径
        std::string klb = ros::package::getPath("image_deal");
        std::string path = klb + "/image_transfer/" + image_name;
        
        // // 绝对路径
        // std::string path = "/home/klb/catkin_ws/catkin_jaka/src/image_deal/image_transfer" + image_name;
        
        cv::imwrite(path.c_str(), img);
	    // cv::imshow("image", img);
        ROS_INFO("image ok!");
	    //cv::waitKey(1);
    }
    else
    {
        ROS_ERROR("no topic image_sub");
    }
 
    ROS_INFO("ok!");
 
    // 多线程，只执行1次
    // ros::MultiThreadedSpinner spinner(0);
    // spinner.spin();
    // // 关闭节点
    // ros::shutdown();

    // 等待一段时间
    ros::Duration wait_success(0.5);
    ROS_INFO("Waiting for a moment ......");
    wait_success.sleep();
 
    return 0;
} 
