
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <GeographicLib/LocalCartesian.hpp>                 //  调用GeographicLib库
#pragma once
class GnssProcess
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //经纬度
    double time;
    double latitude;
    double longitude;
    double altitude;
    //东北地
    double local_E; 
    double local_N;
    double local_U;
    int status;
    int service;

    double origin_longitude;
    double origin_latitude;
    double origin_altitude;
    Eigen::Vector3d pose_cov;
    GnssProcess();
    ~GnssProcess();

    //函数声明
    void init_origin_position(double latitude, double longitude, double altitude);
    void update_xyz(double latitude, double longitude, double altitude);
    void Reverse(const double &local_E, const double &local_N,const double &local_U, double &lat, double &lon, double &alt);

    //外参设置
    void set_extrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot);
    void set_extrinsic(const Eigen::Vector3d &transl);
    void set_extrinsic(const Eigen::Matrix4d &T);
    
    
    private:
    GeographicLib::LocalCartesian geo_converter;//创建Geolib中的对象
    Eigen::Matrix3d Gnss_R_wrt_Lidar;
    Eigen::Vector3d Gnss_T_wrt_Lidar;
};
GnssProcess::GnssProcess()
{
    time = 0.0;
    local_E = 0.0;
    local_N = 0.0;
    local_U = 0.0;
    status = 0 ;
    service = 0;
    origin_altitude = 0;
    origin_latitude = 0;
    origin_latitude = 0;
    pose_cov = Eigen::Vector3d::Zero();
    Gnss_R_wrt_Lidar = Eigen::Matrix3d::Identity();
    Gnss_T_wrt_Lidar = Eigen::Vector3d::Zero();
}
GnssProcess::~GnssProcess(){}
//初始化原始点
void GnssProcess::init_origin_position(double latitude, double longitude, double altitude){
    geo_converter.Reset(latitude, longitude, altitude);
    ROS_INFO("init Gnss OriginPosition");
    origin_altitude = altitude;
    origin_latitude = latitude;
    origin_longitude = longitude;
}
//更新东北地坐标
void GnssProcess::update_xyz(double latitude, double longitude, double altitude){
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GnssProcess::Reverse(const double &local_E, const double &local_N,const double &local_U, double &lat, double &lon, double &alt)
{
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

void GnssProcess::set_extrinsic(const Eigen::Matrix4d &T)
{
    Gnss_T_wrt_Lidar = T.block<3,1>(0,3);
    Gnss_R_wrt_Lidar = T.block<3,3>(0,0);
}
void GnssProcess::set_extrinsic(const Eigen::Vector3d &transl)
{
    Gnss_R_wrt_Lidar.setIdentity();
    Gnss_T_wrt_Lidar = transl;
}
void GnssProcess::set_extrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot)
{
    Gnss_T_wrt_Lidar = transl;
    Gnss_R_wrt_Lidar = rot;
}

