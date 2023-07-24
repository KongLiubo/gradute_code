#include "ros/ros.h"

#include "robot_msgs/Move.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_line_positions");

    ros::NodeHandle n;

    // 封印
    // // ------------从参数服务器中获取目标物体的位置坐标--------
    // float goal_coordinate_x, goal_coordinate_y, goal_coordinate_z;
    
    // bool obtained_flag = 0 ;
    // // 获取目标物体坐标
    // while(!obtained_flag)
    // {   
    //     bool obtained_x_flag = 0, obtained_y_flag = 0, obtained_z_flag = 0;
    //     obtained_x_flag = n.getParam("/world_coordinate_x", goal_coordinate_x);
    //     obtained_y_flag = n.getParam("/world_coordinate_x", goal_coordinate_y);
    //     obtained_z_flag = n.getParam("/world_coordinate_x", goal_coordinate_z);

    //     ROS_INFO("Searching for target parameters");
    //     obtained_flag = (obtained_x_flag&&obtained_y_flag&&obtained_z_flag);
    // }
    // ROS_INFO("target parameters have successfully obationed");
    // // ---------------------------------------------------


    // // ----------通过服务通信将机械臂末端移动到目标物体上空----------------------
    // ros::ServiceClient overhead_move = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");

    // robot_msgs::Move overhead_srv;
    // overhead_srv.request.pose.clear();

    // // 设置运动中的速度与加速度
    // overhead_srv.request.mvvelo = 300;
    // overhead_srv.request.mvacc = 20;
    // // 填充末端目标参数（暂定除了x，y，其余参数初步给固定值）
    // overhead_srv.request.pose.push_back(goal_coordinate_x);
    // overhead_srv.request.pose.push_back(goal_coordinate_y);
    // overhead_srv.request.pose.push_back(goal_coordinate_z);
    // overhead_srv.request.pose.push_back(-0.68);
    // overhead_srv.request.pose.push_back(2.88);
    // overhead_srv.request.pose.push_back(1.103415671);    
    
    // if(overhead_move.call(overhead_srv))
    // {
    //     ROS_INFO("Response from server message: %s",overhead_srv.response.message.c_str());
    //     ROS_INFO("Response from server ret: %d",overhead_srv.response.ret);


    // }else{
    //     ROS_ERROR("failed to call /robot_driver/move_line");
    //     return 1;
    // }
    // // -------------------------------------------------------------



    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");

    robot_msgs::Move srv;
    srv.request.pose.clear();

    srv.request.mvvelo = 10;
    srv.request.mvacc = 2;
    // srv.request.coord_mode = 1;
    // 0~2  单位m， 3~5  单位rad
    srv.request.pose.push_back(0);
    srv.request.pose.push_back(-0.45);
    srv.request.pose.push_back(0.2);
    srv.request.pose.push_back(3.14);
    srv.request.pose.push_back(0);
    srv.request.pose.push_back(0); 

    // srv.request.pose.push_back(-0.427491);
    // srv.request.pose.push_back(0.135083);
    // srv.request.pose.push_back(0.605524);
    // srv.request.pose.push_back(1.928094);
    // srv.request.pose.push_back(2.492005);
    // srv.request.pose.push_back(-0.398228);

    // srv.request.pose.push_back(0.111254);
    // srv.request.pose.push_back(-0.665768);
    // srv.request.pose.push_back(0.114838);
    // srv.request.pose.push_back(-92.466/180*PI);
    // srv.request.pose.push_back(-2.542/180*PI);
    // srv.request.pose.push_back(-173.366/180*PI);

    //cout<<"ssssssssssssss"<<endl;
// pose: [-719.2,21.096,552.796,-0.68,2.88,1.01]
// has_ref: false
// ref_joint: [0]
// mvvelo: 20.0
// mvacc: 10.0
// mvtime: 0.0
// mvradii: 0.0
// coord_mode: 0
// index: 0

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);


    }else{
        ROS_ERROR("failed to call /robot_driver/move_line");
        return 1;
    }




    return 0;

}
