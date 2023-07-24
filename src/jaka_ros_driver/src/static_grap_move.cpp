#include "ros/ros.h"
#include "robot_msgs/Move.h"
#include "string"
#include "libs/robot.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

#include "vector"

#include "ctime"

using namespace std;
const double PI = 3.1415926;


/*
@desc - 假闭环，使jakazu3到达目标位置（笛卡尔运动）

@param - ros::ServiceClient& grap_client  话题发布者
         robot_msgs::Move& grap_cmd_data  话题发布的信息

@returns - bool  返回1 出错
                 返回0 运行正常

@throws - 描述函数可以抛出的错误类型

@example - 包含一个或多个显示输入和预期输出的示例。
*/
bool moveline_set_pose(ros::ServiceClient& grap_client, robot_msgs::Move& grap_cmd_data)
{
    while(1)
    {
        boost::shared_ptr<geometry_msgs::TwistStamped const> tool_sub;
        tool_sub = ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/robot_driver/tool_point", ros::Duration(3));
        ros::Duration wait_move(0.01);
        
        if(tool_sub != NULL)
        {
            geometry_msgs::TwistStamped tool_raw = *tool_sub; 
            if((fabs(tool_raw.twist.linear.x/grap_cmd_data.request.pose[0] - 1)<0.01)
                &&(fabs(tool_raw.twist.linear.y/grap_cmd_data.request.pose[1] - 1)<0.01)
                &&(fabs(tool_raw.twist.linear.z/grap_cmd_data.request.pose[2] - 1)<0.01))
            {
                break;
            }
            else
            {
                wait_move.sleep();
                // 发送到机械臂中
                if(grap_client.call(grap_cmd_data))
                {
                    ROS_INFO("Response from server message: %s",grap_cmd_data.response.message.c_str());
                    ROS_INFO("Response from server ret: %d",grap_cmd_data.response.ret);
                }
                else
                {
                    ROS_ERROR("failed to call /robot_driver/move_line");
                    return 1;
                }                 
            }

        }

    }
    return 0;
}


int main(int argc, char *argv[])
{
    system("rosrun image_deal static_image_to_coordinate.py");
    // system("rosnode kill /hk_camera_node");
    // 初始化节点，句柄
    ros::init(argc,argv,"static_grap_move");
    ros::NodeHandle nh;

    // 随机数（为最终放置准备）
    srand(time(0));
    // 缓冲时间
    ros::Duration wait_buffering(0.5);

    // 从参数服务器中获取目标物体的位姿信息
    float goal_coordinate_x, goal_coordinate_y, goal_coordinate_z, goal_rotate_rz;
    float set_coordinate_z = 14;

    nh.getParam("/world_coordinate_x",goal_coordinate_x);
    nh.getParam("/world_coordinate_y",goal_coordinate_y);
    nh.getParam("/world_coordinate_z",goal_coordinate_z);
    nh.getParam("/rotate_angle_rz",goal_rotate_rz);
    // 对信息进行二次处理
    goal_coordinate_x = goal_coordinate_x/1000.0;
    goal_coordinate_y = goal_coordinate_y/1000.0;

    goal_coordinate_z = set_coordinate_z;
    goal_coordinate_z = goal_coordinate_z/1000.0;
    goal_rotate_rz = goal_rotate_rz/180*PI;
    // 于终端中打印出来
    ROS_INFO("goal_coordinate_x = %f", goal_coordinate_x);
    ROS_INFO("goal_coordinate_y = %f", goal_coordinate_y);
    ROS_INFO("goal_coordinate_z = %f", goal_coordinate_z);
    ROS_INFO("goal_rotate_rz = %f", goal_rotate_rz);   
    
    /*------1. 根据该信息，把机械臂移动过去----------*/ 
    // 该部分应包含两部分，部分1：移动到上空，下降到指定位置
    ros::ServiceClient grap_client = nh.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    // 需要发送的客户端class
    robot_msgs::Move grap_cmd_data;
    grap_cmd_data.request.pose.clear();
    // 设定速度与加速度
    grap_cmd_data.request.mvvelo = 3;
    grap_cmd_data.request.mvacc = 0.4;
    // 设置末端位姿信息
    // 0~2  单位m， 3~5  单位rad
    std::vector<float> wait_position;
    float temp_coordinate_z = goal_coordinate_z+0.04;

    wait_position.push_back(goal_coordinate_x);
    wait_position.push_back(goal_coordinate_y);
    wait_position.push_back(temp_coordinate_z);
    wait_position.push_back(-3.14);
    wait_position.push_back(0);
    wait_position.push_back(0);

    grap_cmd_data.request.pose = wait_position;   

    // 等待机械臂到达目标位置
    ROS_INFO("Waiting for the operation implementation ......");

    // API封装（暂时封印）
    if(moveline_set_pose(grap_client, grap_cmd_data))
    {
        return 1;
    }
    else
    {
        ROS_INFO("I has reached the designated position!");
    }

    // 每个阶段实现后，缓冲1s
    ROS_INFO("Waiting for a moment ......");
    wait_buffering.sleep(); 

    /*-------2. 机械臂下降，实现对末端关节角度的控制-------*/ 
    wait_position[2] = goal_coordinate_z;
    grap_cmd_data.request.pose = wait_position; 

    // 等待机械臂到达目标位置
    ROS_INFO("Waiting for the operation implementation ......");
    // API封装（暂时封印）
    if(moveline_set_pose(grap_client, grap_cmd_data))
    {
        return 1;
    }
    else
    {
        ROS_INFO("I has reached the designated position!");
    }

    // 每个阶段实现后，缓冲1s
    ROS_INFO("Waiting for a moment ......");
    wait_buffering.sleep(); 

    /*-------3. 获取关节角度信息，实现对末端关节角度的控制-------*/ 
    // 获得关节角信息
    ROS_INFO("grapper is ratating ......");
    boost::shared_ptr<sensor_msgs::JointState const> joints_sub;
    joints_sub = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot_driver/joint_states", ros::Duration(3));
    // 获取当前关节角信息
    if(joints_sub != NULL)
    {
        sensor_msgs::JointState joints_raw = *joints_sub;  
        // 关节角处理（均为rad）
        // joints_raw.position[5] += goal_rotate_rz;
        // float set_end_joint = joints_raw.position[5] + 1;  //test
        float set_end_joint = joints_raw.position[5] - goal_rotate_rz;
        // 末端关节角旋转
        ros::ServiceClient joints_client = nh.serviceClient<robot_msgs::Move>("/robot_driver/move_joint");  
        robot_msgs::Move joints_srv;

        joints_srv.request.pose.clear();
        joints_srv.request.mvvelo = 5;
        joints_srv.request.mvacc = 1;

        joints_srv.request.pose.push_back(joints_raw.position[0]);
        joints_srv.request.pose.push_back(joints_raw.position[1]);
        joints_srv.request.pose.push_back(joints_raw.position[2]);
        joints_srv.request.pose.push_back(joints_raw.position[3]);
        joints_srv.request.pose.push_back(joints_raw.position[4]);
        joints_srv.request.pose.push_back(set_end_joint);        

        // 等待机械臂末端旋转到目标位置
        ROS_INFO("Waiting for the operation implementation ......");
        while(1)
        {
            boost::shared_ptr<sensor_msgs::JointState const> end_joint_sub;
            end_joint_sub = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot_driver/joint_states", ros::Duration(3));
            ros::Duration wait_ratate(0.1);

            if(end_joint_sub != NULL)
            {
                sensor_msgs::JointState end_joint_raw = *end_joint_sub; 
                if(fabs(end_joint_raw.position[5]/set_end_joint - 1)<0.01)
                {
                    break;
                }
                else
                {   
                    // 执行
                    if(joints_client.call(joints_srv))
                    {
                        ROS_INFO("Response from server message: %s",joints_srv.response.message.c_str());
                        ROS_INFO("Response from server ret: %d",joints_srv.response.ret);            
                    }
                    else
                    {
                        ROS_ERROR("failed to call /robot_driver/move_line");
                        return 1;
                    }
                    wait_ratate.sleep();
                }
            }
            else
            {
                ROS_ERROR("no topic end_joint_sub");
                return 1;
            }
        }  
    }
    else
    {
        ROS_ERROR("no topic joints_sub");
        return 1;
    }
    ROS_INFO("I has reached the designated position!");

    // 每个阶段实现后，缓冲1s
    ROS_INFO("Waiting for a moment ......");
    wait_buffering.sleep();


    /* -----------4. 夹爪闭合------------- */
    system("rosrun dh_hand_driver hand_controller_client 1 2 90");

    /* -----------5. 机械臂移动到另一位置（放置物体位置）--------- */
    // 基础值， 单位mm
    float base_center_x = 80, base_center_y = -440;
    int scope_x_y = 50;
    wait_position[0] = (base_center_x + rand()%scope_x_y)/1000.0;
    wait_position[1] = (base_center_y - rand()%scope_x_y)/1000.0;
    wait_position[2] = 0.087;
    wait_position[3] = -3.14;
    wait_position[4] = 0;
    wait_position[5] = 0;
    grap_cmd_data.request.pose = wait_position; 

    ROS_INFO("end_x: %f, end_y: %f",wait_position[0], wait_position[1]);

    // 等待机械臂到达目标位置
    ROS_INFO("Waiting for the operation implementation ......");

    if(moveline_set_pose(grap_client, grap_cmd_data))
    {
        return 1;
    }
    else
    {
        ROS_INFO("I has reached the designated position!");
    }

    // 每个阶段实现后，缓冲1s
    ROS_INFO("Waiting for a moment ......");
    wait_buffering.sleep(); 

    /* -----------6. 夹爪张开，放置物体------------ */
    system("rosrun dh_hand_driver hand_controller_client 1 90 90");
    /* -----------7. 机械臂回到终止位置------------ */

}