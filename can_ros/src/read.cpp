#include"ros/ros.h"
#include<stdio.h>
#include"can_ros/send_can.h"
#include"can_ros/read_can.h"
#include"geometry_msgs/PoseStamped.h"
//测试：读取到ros发送到can的消息的回调函数
// void send_function(const can_ros::send_can& msg)
// {
//     ROS_INFO("I send r1:[%f]",msg.send_steering_r1*180/3.14);
//     ROS_INFO("I send l1:[%f]",msg.send_steering_l1*180/3.14);
//     ROS_INFO("I send r2:[%f]",msg.send_steering_r2*180/3.14);
//     ROS_INFO("I send l2:[%f]",msg.send_steering_l2*180/3.14);
//     ROS_INFO("I send r3:[%f]",msg.send_steering_r3*180/3.14);
//     ROS_INFO("I send l3:[%f]",msg.send_steering_l3*180/3.14);
//     ROS_INFO("I send throttle_r:[%f]",msg.send_throttle_r);
//     ROS_INFO("I send throttle_l:[%f]",msg.send_throttle_l);
// }
// void ndt_function(const geometry_msgs::PoseStamped& msg)
// {
//     ROS_INFO("I ndt r1:[%f]",msg.pose.position.x);
//     ROS_INFO("I ndt l1:[%f]",msg.pose.position.y);

// }
//测试：读取到can发送给ros的消息的回调函数
void send_function1(const can_ros::read_can& msg)
{
    ROS_INFO("I read angle r1:[%f]",msg.read_steering_r1);
    ROS_INFO("I read angel l1:[%f]",msg.read_steering_l1);
    ROS_INFO("I read angle r2:[%f]",msg.read_steering_r2);
    ROS_INFO("I read angel l2:[%f]",msg.read_steering_l2);
    ROS_INFO("I read angel r3:[%f]",msg.read_steering_r3);
    ROS_INFO("I read angel l3:[%f]",msg.read_steering_l3);
    ROS_INFO("I read speed r1:[%f]",msg.read_speed_r1);
    ROS_INFO("I read speed l1:[%f]",msg.read_speed_l1);
    ROS_INFO("I read speed r2:[%f]",msg.read_speed_r2);
    ROS_INFO("I read speed l2:[%f]",msg.read_speed_l2);
    ROS_INFO("I read speed r3:[%f]",msg.read_speed_r3);
    ROS_INFO("I read speed l3:[%f]",msg.read_speed_l3);
}


int main(int argc,char **argv)
{   
    ros::init(argc,argv,"read");
    ros::NodeHandle can_read;//创建句柄
    //ros::Subscriber sub_r = can_read.subscribe("/decision/steering_angle",10,send_function);//创建发布者，话题/decision/steering_angle，接受发送给底盘数据
    //ros::Subscriber sub_ndt = can_read.subscribe("/ndt_pose",10,ndt_function);
    ros::Subscriber sub1 = can_read.subscribe("/can_data",10,send_function1);//创建发布者，话题/can_data，接受数据，接受底盘发布的数据
    ros::spin();
    return 0;
}
