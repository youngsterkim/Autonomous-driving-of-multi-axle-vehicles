#include"ros/ros.h"
#include<stdio.h>
#include<cmath>
#include<Eigen/Core>
#include<Eigen/Eigen>
#include "Position_state.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#pragma once
class Preview_error
{
    public:
    nav_msgs::Path path;
    fast_lio::Position_state position_state;
    float yaw_v = 0;
    float yaw_v_error = 0;
    float d_yaw_v_error = 0;
    float i_yaw_v_error = 0;
    float vx;

    float beta = 0;
    float beta_error = 0;
    float d_beta_error = 0;
    float i_beta_error = 0;
    bool idx_over = false;

    float Preview_time = 0;
    //预瞄横向长度
    float Preview_dis_y = 0;
    //预瞄纵向长度
    float Preview_dis_x = 0;
    float Preview_dis = 0;

    float tar_yaw_v = 0;
    float tar_beta = 0;
    int idx = 0;//预瞄点的索引
    float min = 0;//用于dist[]
    //预瞄长度
    float ld = 0.8;
    //初始化，输入轨迹与坐标
    void Preview_error_init(nav_msgs::Path& path_in,fast_lio::Position_state& position_state_in,float &dt_in,bool &use_lla_in);
    //计算预瞄距离、预瞄时间
    void Preview_time_dis();
    //计算理想横摆角速度及其误差、微分、积分、质心侧偏角及其误差微分、积分
    void error_d_i();
    float dt = 0;
    float dt_array_yaw_now = 0;
    float dt_array_yaw_last = 0;
    float dt_array_beta_last = 0;
    float dt_array_beta_now = 0;
    float d_yaw_v_error_array_last = 0;
    float d_yaw_v_error_array_now = 0;
    float d_beta_error_array_last = 0;
    float d_beta_error_array_now = 0;
    int count_d_yaw = 0;
    int count_d_beta = 0;
    float dot_wd = 0;
    float dot_beta = 0;
    float d_wd_array_last = 0;
    float d_wd_array_now = 0;
    float dt_array_wd_last = 0;
    float dt_array_wd_now = 0;
    int count_d_wd = 0;

    float a = 1.058;
    float b = 0.025;
    float c = -0.958;
    float m = 375;
    float L = 2.016;
    float k1 = -13318*1.15;
    float k2 = -14560*1.15;
    float k3 = -16169*1.15;
    float previer_angle = 0;
    bool use_lla;
};
void Preview_error::Preview_error_init(nav_msgs::Path& path_in,fast_lio::Position_state &position_state_in,float &dt_in, bool &use_lla_in)
{
    path = path_in;
    position_state = position_state_in;
    dt = dt_in;
    yaw_v = position_state_in.state_wx;
    use_lla = use_lla_in;
}
void Preview_error::Preview_time_dis()
{
    int n = path.poses.size();
    float dist[n];
    float pos[1][2];
    pos[0][0] = position_state.Positon_x;
    pos[0][1] = position_state.Positon_y;

    //搜索预瞄点
    //先射第一个点
    dist[0] = sqrt(pow(path.poses[0].pose.position.x-pos[0][0],2)+pow(path.poses[0].pose.position.y - pos[0][1],2));
    min = dist[0];
    for(int i = 1; i< path.poses.size(); i++)
    {
        //找到距离最近的点及标签
        dist[i] =sqrt(pow(path.poses[i].pose.position.x-pos[0][0],2)+pow(path.poses[i].pose.position.y - pos[0][1],2));
        if(dist[i] < min)
        {
            min = dist[i];
        }
    }
    for(int i = 0; i <path.poses.size(); i++)
    {
        if(dist[i] == min)
        {
            idx = i;
            break;
        }
    }
    //根据车辆最近点向前方找最近的点
    float l_step = 0;
    while(l_step < ld & idx < path.poses.size())
    {
        l_step = l_step + sqrt(pow(path.poses[idx].pose.position.x-path.poses[idx+1].pose.position.x,2)
                            + pow(path.poses[idx].pose.position.y-path.poses[idx+1].pose.position.y,2));
        idx = idx + 1;
    }
    //求解车身与预瞄点的夹角
    if (use_lla == true)
    {
        previer_angle = atan2((path.poses[idx].pose.position.y-pos[0][1]),(path.poses[idx].pose.position.x-pos[0][0]))-position_state.rot_z + 3.1415/2 ;
    }
    else{
        previer_angle = atan2((path.poses[idx].pose.position.y-pos[0][1]),(path.poses[idx].pose.position.x-pos[0][0]))-position_state.rot_z ;
    }
    if (previer_angle > 3.1415/2)previer_angle = previer_angle - 3.1415;
    if (previer_angle < -3.1415/2)previer_angle = previer_angle + 3.1415;
    if (previer_angle > 3.1415/4 || previer_angle < -3.1415/4)ROS_INFO("back");
    if (previer_angle < 3.1415/4 && previer_angle > -3.1415/4)ROS_INFO("forwork");
    //横向预瞄距离
    Preview_dis_y = dist[idx]*sin(previer_angle);
    //纵向预瞄距离
    Preview_dis_x = dist[idx]*cos(previer_angle);
    Preview_dis = dist[idx];
    ROS_INFO("pre_x = %f ,pre_y = %f,previer_angle = %f",Preview_dis_x,Preview_dis_y,previer_angle*180/3.1415);
    ROS_INFO("idx = %d,dist = %f",idx,dist[idx]);
    if (idx >= path.poses.size()-1 )idx_over = true;
    //预瞄时间
    Preview_time = Preview_dis_x/position_state.state_vx;
}
void Preview_error::error_d_i()
{
    //理想角速度
    if(abs(position_state.state_vx) < 0.1){
        tar_yaw_v = 0;
    }
    else{

        tar_yaw_v = 2*(atan(Preview_dis_y/position_state.state_vx/Preview_time)-position_state.beta)/Preview_time;
    }
    // ROS_INFO("tar_yaw_v = %f",tar_yaw_v);
    //横摆角速度误差
    yaw_v_error = yaw_v - tar_yaw_v;
    // ROS_INFO("yaw_v_error = %f",yaw_v_error);
    //质心侧偏角误差
    beta_error = position_state.beta - tar_beta;
    if (abs(beta_error) > 3.14)
    {
        beta_error = 0;
    }

    // ROS_INFO("beta_error = %f",beta_error);
    //横摆角速度微分dot_wd
    if(dt == 0)
    {
        dot_wd = 0;
    }
    else{

        if(count_d_wd < 5){
            d_wd_array_now = d_wd_array_now + yaw_v;
            dt_array_wd_now = dt_array_wd_now + dt;
            count_d_wd++;
        }
        else{
            d_wd_array_now = (d_wd_array_now + yaw_v)/5;
            dt_array_wd_now = dt_array_wd_now + dt;
            //横摆角速度误差微分计算
            dot_wd = (d_wd_array_now - d_wd_array_last)/((dt_array_wd_now + dt_array_wd_last)/2);
        
            //重新复位
            d_wd_array_last = d_wd_array_now;
            dt_array_wd_last = dt_array_wd_now;
            dt_array_wd_now = 0;
            d_wd_array_now = 0;
            count_d_wd = 0;
        }
        
    }

    //横摆角速度误差微分
    if(dt == 0)
    {
        d_yaw_v_error = 0;
    }
    else{

        if(count_d_yaw < 5){
            d_yaw_v_error_array_now = d_yaw_v_error_array_now + yaw_v_error;
            dt_array_yaw_now = dt_array_yaw_now + dt;
            count_d_yaw++;
        }
        else{
            d_yaw_v_error_array_now = (d_yaw_v_error_array_now + yaw_v_error)/5;
            dt_array_yaw_now = dt_array_yaw_now + dt;
            //横摆角速度误差微分计算
            d_yaw_v_error = (d_yaw_v_error_array_now - d_yaw_v_error_array_last)/((dt_array_yaw_now + dt_array_yaw_last)/2);
        
            //重新复位
             d_yaw_v_error_array_last = d_yaw_v_error_array_now;
            dt_array_yaw_last = dt_array_yaw_now;
            dt_array_yaw_now = 0;
            d_yaw_v_error_array_now = 0;
            count_d_yaw = 0;
        }
        
    }

    //横摆角速度误差积分
    if (abs(yaw_v_error) > 3.14)
    {
        yaw_v_error = 0;
    }
    i_yaw_v_error = i_yaw_v_error + yaw_v_error*dt;
    
    //质心侧偏角误差微分
    if(dt == 0)
    {
        d_beta_error = 0;
    }
    else{

        if(count_d_beta < 5){
            d_beta_error_array_now = d_beta_error_array_now + beta_error;
            dt_array_beta_now = dt_array_beta_now + dt;
            count_d_beta++;

        }
        else{
            d_beta_error_array_now = (d_beta_error_array_now + beta_error)/5;
            dt_array_beta_now = dt_array_beta_now + dt;
            //质心侧偏角误差微分计算
            d_beta_error = (d_beta_error_array_now - d_beta_error_array_last)/((dt_array_beta_last + dt_array_beta_now)/2);
            dot_beta = d_beta_error;
            //重新复位
            d_beta_error_array_last = d_beta_error_array_now;
            dt_array_beta_last = dt_array_beta_now;
            dt_array_beta_now = 0;
            d_beta_error_array_now = 0;
            count_d_beta = 0;
        }
        
    }
    //质心侧偏角误差积分

    i_beta_error = i_beta_error + beta_error*dt;

    //打印模块所有信息
    ROS_INFO("/************Preview***********************/");
    ROS_INFO("i_beta_error = %f",i_beta_error);
    ROS_INFO("d_beta_error = %f",d_beta_error);
    ROS_INFO("i_yaw_v_error = %f",i_yaw_v_error);
    ROS_INFO("d_yaw_v_error = %f",d_yaw_v_error);
    ROS_INFO("yaw_v_error = %f",yaw_v_error);
    ROS_INFO("tar_yaw_v = %f",tar_yaw_v);
    ROS_INFO("beta_error = %f",beta_error);
    ROS_INFO("dot_wd = %f",dot_wd);
    ROS_INFO("yaw_v= %f",yaw_v);
    ROS_INFO("dot_beta = %f",dot_beta);
    ROS_INFO("d_yaw_v_error_array_last = %f",d_yaw_v_error_array_last);
    ROS_INFO("d_yaw_v_error_array_now = %f",d_yaw_v_error_array_now);
    ROS_INFO("position_state.state_vx = %f",position_state.state_vx);
}
