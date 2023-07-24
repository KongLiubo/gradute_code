#include "ros/ros.h"

#include "robot_msgs/Move.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_joint_positions");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_joint");

    robot_msgs::Move srv;
    srv.request.pose.clear();
    srv.request.mvvelo = 20;
    srv.request.mvacc = 5;
    srv.request.coord_mode = 0;
//    srv.request.pose.push_back(18.909*PI/180);
//    srv.request.pose.push_back(100.932*PI/180);
//    srv.request.pose.push_back(65.263*PI/180);
//    srv.request.pose.push_back(114.719*PI/180);
//    srv.request.pose.push_back(-90.576*PI/180);
//    srv.request.pose.push_back(-31.437*PI/180);
    /* srv.request.pose.push_back(0*PI/180);
    srv.request.pose.push_back(20*PI/180);
    srv.request.pose.push_back(0*PI/180);
    srv.request.pose.push_back(0*PI/180);
    srv.request.pose.push_back(0*PI/180);
    srv.request.pose.push_back(0*PI/180); */

    srv.request.pose.push_back(118*PI/180);
    srv.request.pose.push_back(178.106*PI/180);
    srv.request.pose.push_back(-51.110*PI/180);
    srv.request.pose.push_back(126.005*PI/180);
    srv.request.pose.push_back(247*PI/180);
    srv.request.pose.push_back(150*PI/180);
    // srv.request.pose.push_back(82.073/1000);
    // srv.request.pose.push_back(-602.6/1000);
    // srv.request.pose.push_back(139.903/1000);
    // srv.request.pose.push_back(167.7*PI/180);
    // srv.request.pose.push_back(2.591*PI/180);
    // srv.request.pose.push_back(4.052*PI/180); 

    //cout<<"ssssssssssssss"<<endl;

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
