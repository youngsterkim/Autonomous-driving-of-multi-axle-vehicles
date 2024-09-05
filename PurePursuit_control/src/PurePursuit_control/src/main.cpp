#include"ros/ros.h"
#include<stdio.h>
#include<cmath>
#include"send_can.h"
#include"read_can.h"
#include "Position_state.h"
#include "Preview_error.h"
#include "purepursuit_controler.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include "gnss_process.hpp"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
// //全局变量设置
float send_steering_r1 = 0;
float send_steering_l1 = 0;
float send_steering_r2 = 0;
float send_steering_l2 = 0;
float send_steering_r3 = 0;
float send_steering_l3 = 0;
float L1 = 1.033;
float L2 = 0.983;
float W = 1.671;
float R = 0;
bool use_lla = true;
int i = 0;
int count = 0;
fast_lio::Position_state position_state;//设置定位信息
nav_msgs::Path path;//轨迹队列
//车辆可视化消息类型
visualization_msgs::Marker car_cube;
//车辆可行区域可视化消息
visualization_msgs::Marker safety_region;
GnssProcess gnss_path;
GnssProcess gnss_get_enu;
bool gnss_init = false;
//接收定位模块回调函数(包含定位信息以及质心侧偏角、横摆角速度)
void position_state_cbk(const fast_lio::Position_state::ConstPtr& position_state_msg)
{
    position_state = *position_state_msg;
    if (use_lla == true){
        if (!gnss_init) 
        {
            gnss_path.init_origin_position(position_state.Positon_x, position_state.Positon_y, 0);
            gnss_get_enu.init_origin_position(position_state.Positon_x, position_state.Positon_y, 0);
            gnss_init = true;
        }
        if ( gnss_init == true)
        {
                gnss_get_enu.update_xyz(position_state.Positon_x,position_state.Positon_y, 0);
                position_state.Positon_x = gnss_get_enu.local_E;
                position_state.Positon_y = gnss_get_enu.local_N;
        }
    }
}
//局部路径回调
void local_path_cbk(const nav_msgs::Path::ConstPtr& local_path_in)
{
    path = *local_path_in;
    ROS_INFO("111%d",path.poses.size());
}
//根据阿克曼转角算出第二三轴转角轴的转角、前后轮转向
void Akm_get_delta(double& input_steer)
{
        if (input_steer > 20)input_steer = 20;
        if (input_steer < -20)input_steer = -20;
        R = L1/tan(input_steer);
        send_steering_r1 = input_steer*180/3.1415;
        send_steering_l1 = atan(L1/(R-W))*180/3.1415;
        send_steering_r3 = -atan(L2/R)*180/3.1415;
        send_steering_l3 = -atan(L2/(R- W))*180/3.1415;
        send_steering_l2 = 0;
        send_steering_l2 = 0;
}
//设置车辆可视化cube
void car_cube_get()
{
    car_cube.type = visualization_msgs::Marker::CUBE;
    car_cube.action = visualization_msgs::Marker::ADD;
    car_cube.ns = "car_cube";
    car_cube.id = 0;
    car_cube.header.frame_id = "body";
    // 设置 Marker 的位置
    car_cube.pose.position.x = -0.5;
    car_cube.pose.position.y = 0.0;
    car_cube.pose.position.z = -0.55;
    // 设置 Marker 的方向
    car_cube.pose.orientation.x = 0.0;
    car_cube.pose.orientation.y = 0.0;
    car_cube.pose.orientation.z = 0.0;
    car_cube.pose.orientation.w = 1.0;
    // 设置 Marker 的尺寸
    car_cube.scale.x = 2.6;
    car_cube.scale.y = 1.6;
    car_cube.scale.z = 1.1;
    // 设置 Marker 的颜色
    car_cube.color.r = 0.0f;
    car_cube.color.g = 1.0f;
    car_cube.color.b = 0.0f;
    car_cube.color.a = 1.0;
}
int main(int argc,char *argv[])
{
    ros::init(argc,argv,"control");
    ros::NodeHandle n;
    can_ros::send_can msg;
    //实例化position_state订阅者对象
    ros::Subscriber sub = n.subscribe<fast_lio::Position_state>("/state_to_control",1,position_state_cbk);
    //局部轨迹回调函数
    ros::Subscriber local_path_sub = n.subscribe<nav_msgs::Path>("/local_path",1,local_path_cbk);

    ros::Publisher pubcan = n.advertise<can_ros::send_can>("/decision/steering_angle",1000);
    //实例化车辆本体对象，用于可视化
    ros::Publisher pubcar_cube = n.advertise<visualization_msgs::Marker>("car_cube",1);
    //创建预瞄估计实例；
    Preview_error Preview_error_;
    //创建纯跟踪实例
    purepursuit_controler purepursuit_controler_;
    //用于计算时间dt
    ros::Time t_last = ros::Time::now() ;
    ros::Time t_now = ros::Time::now() ;
    float dt = 0;
    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        if (!path.poses.empty())
        {
            t_now = ros::Time::now();
            dt = (t_now-t_last).toSec();
            /********************根据定位及轨迹信息等，计算预瞄距离、预瞄时间，并进一步获取理想横摆角速度和理想质心侧偏角,再一步获取质心侧偏角误差及微分、横摆角速度误差及微分************/
            //预瞄实例初始化
            Preview_error_.Preview_error_init(path,position_state,dt, use_lla);
            //计算预瞄参数
            Preview_error_.Preview_time_dis();
            //纯跟踪控制器初始化
            purepursuit_controler_.purepursuit_controler_init(Preview_error_);
            //计算转角
            purepursuit_controler_.purepursuit_controler_delta();
            /*根据阿克曼转向算出第二轴转角*/
            Akm_get_delta(purepursuit_controler_.delta1);
            if ( Preview_error_.idx_over == true)
            {
                ROS_INFO("IDX_OVER = %d",Preview_error_.idx_over);
                msg.send_steering_r1 = 0;
                msg.send_steering_l1 = 0;
                msg.send_steering_r2 = 0;
                msg.send_steering_l2 = 0;
                msg.send_steering_r3 = 0;
                msg.send_steering_l3 = 0;
                msg.send_throttle_r = 0;
                msg.send_throttle_l = 0;
            }
            else{
                /*输出转角信息控制底盘*/
                msg.send_steering_r1 = send_steering_r1;
                msg.send_steering_l1 = send_steering_l1;
                msg.send_steering_r2 = send_steering_r2;
                msg.send_steering_l2 = send_steering_l2;
                msg.send_steering_r3 = send_steering_r3;
                msg.send_steering_l3 = send_steering_l3;
                msg.send_throttle_r = 0;
                msg.send_throttle_l = 0;
            }
        }
        else{
            msg.send_throttle_r = 0;
            msg.send_throttle_l = 0;
        }
        /*输出转角信息控制底盘*/
        pubcan.publish(msg);//发布底盘消息
        //发布car_cube可视化消息
        car_cube_get();
        pubcar_cube.publish(car_cube);
        //发布轨迹可视化
        ROS_INFO("send_steering_r1 = %f",msg.send_steering_r1);
        ROS_INFO("send_steering_r2 = %f",msg.send_steering_r2);
        ROS_INFO("send_steering_r3 = %f",msg.send_steering_r3);
        ROS_INFO("send_throttle_l= %f",msg.send_throttle_l);
        r.sleep();
    }
    ROS_INFO("down");
    return 0;
}