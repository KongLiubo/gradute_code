#include "ros/ros.h"
#include "robot_msgs/Move.h"
#include "string"
#include "libs/robot.h"
#include "std_msgs/Float32MultiArray.h"

// #include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"

// #include "sensor_msgs/JointState.h"
// #include "geometry_msgs/TwistStamped.h"

using namespace std;
const double PI = 3.1415926;

// void test_callback(const geometry_msgs::TwistStamped& tool_point, const sensor_msgs::JointState& joint_states)
// {
//     ROS_INFO("I heard: linear[%f, %f, %f] angular[%f, %f, %f]", 
//     tool_point.twist.linear.x, tool_point.twist.linear.y, tool_point.twist.linear.z,
//     tool_point.twist.angular.x, tool_point.twist.angular.y, tool_point.twist.angular.z);

//     ROS_INFO("I heard: joint1:[%f], joint2:[%f], joint3:[%f], joint4:[%f], joint5:[%f], joint6:[%f]", 
//     joint_states.position[0], joint_states.position[1], joint_states.position[2],
//     joint_states.position[3], joint_states.position[4], joint_states.position[5]);

// }


void grap_callback(const std_msgs::Float32MultiArray::ConstPtr &grap_data)
{
    float goal_coordinate_x, goal_coordinate_y, goal_coordinate_z, goal_rotate_rz;
    // 从话题中获取位置坐标
    goal_coordinate_x = grap_data->data[0]/1000.0;
    goal_coordinate_y = grap_data->data[1]/1000.0;
    goal_coordinate_z = grap_data->data[2]/1000.0;
    goal_rotate_rz = grap_data->data[3]/180*PI;

    // ROS_INFO("goal_coordinate_x = %f", goal_coordinate_x);
    // ROS_INFO("goal_coordinate_y = %f", goal_coordinate_y);
    // ROS_INFO("goal_coordinate_z = %f", goal_coordinate_z);
    // ROS_INFO("goal_rotate_rz = %f", goal_rotate_rz);

    // system("rosnode kill image_to_coordinate");

    // 实例化客户端
    ros::NodeHandle temp_nh;
    ros::ServiceClient grap_client = temp_nh.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    
    
    // 需要发送的客户端class
    robot_msgs::Move grap_cmd_data;
    grap_cmd_data.request.pose.clear();

    // 设定速度与加速度
    grap_cmd_data.request.mvvelo = 2;
    grap_cmd_data.request.mvacc = 0.2;

    // 设置末端位姿信息
    // 0~2  单位m， 3~5  单位rad
    grap_cmd_data.request.pose.push_back(-0.2137);
    grap_cmd_data.request.pose.push_back(-0.4867);
    grap_cmd_data.request.pose.push_back(0.0814);
    grap_cmd_data.request.pose.push_back(-3.14);
    grap_cmd_data.request.pose.push_back(0);
    grap_cmd_data.request.pose.push_back(0);

    if(grap_client.call(grap_cmd_data))
    {
        ROS_INFO("Response from server message: %s",grap_cmd_data.response.message.c_str());
        ROS_INFO("Response from server ret: %d",grap_cmd_data.response.ret);
    }
    else
    {
        ROS_ERROR("failed to call /robot_driver/move_line");
    }  
   
}



int main(int argc, char *argv[])
{
    // 初始化节点，句柄
    ros::init(argc,argv,"grap_move");
    ros::NodeHandle nh;

    // //ros::Rate wait(0.5);
    // system("gnome-terminal -- bash -c 'source ~/Jaka_ROS/jaka_ros_driver_cpp/devel/setup.bash; roslaunch jaka_ros_driver start.launch'&");
    // //wait.sleep();
    // 实例化话题接受方
    ros::Subscriber grap_sub = nh.subscribe<std_msgs::Float32MultiArray>("/goal_world_coordinate", 10, grap_callback);

    // message_filters::Subscriber<std_msgs::Float32MultiArray> tool_sub(nh, "/goal_world_coordinate", 1);
    // message_filters::Subscriber<sensor_msgs::JointState> joint_sub(nh, "/robot_driver/joint_states", 1);

    // message_filters::TimeSynchronizer<std_msgs::Float32MultiArray, sensor_msgs::JointState> sync(tool_sub, joint_sub, 10);
    // sync.registerCallback(boost::bind(&test_callback, _1, _2));
    // 先运行一次
    ros::spin();

    return 0;
}
