#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/image_encodings.h"


int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "image_deal");
    ros::NodeHandle nh;

    // std::string demo;

    // std::string world_coordinate_x = "/world_coordinate_x_";
    // std::string world_coordinate_y = "/world_coordinate_y_";
    // std::string world_coordinate_z = "/world_coordinate_z_";

    // int a = 0;

    // std::stringstream world_coordinate_x_demo;
    // std::stringstream world_coordinate_y_demo;
    // std::stringstream world_coordinate_z_demo;

    // world_coordinate_x_demo << world_coordinate_x << a;

    // demo = world_coordinate_x_demo.str();

    


    // ROS_INFO("demo = %s", demo.c_str());

    std::vector<float> stus;

    ros::param::get("list_world_coordinate_pose", stus);

    ROS_INFO("%f", stus[0]);
    


    return 0;
}
