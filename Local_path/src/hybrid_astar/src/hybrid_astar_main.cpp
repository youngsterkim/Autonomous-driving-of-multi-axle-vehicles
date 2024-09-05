
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "hybrid_astar_searcher.h"
#include "gnss_process.hpp"
#include "Position_state.h"
int ddd = 0;
nav_msgs::OccupancyGrid local_map;
/*********环境地图参数*********/
double grid_width = 60;
double grid_height = 40;
double grid_origin_x = -10;
double grid_origin_y = -10;
double grid_resolution = 0.5;
/**********车辆代价参数************/
double steering_angle = 20;
int steering_angle_discrete_num = 10;
double wheel_base = 1.0;
double segment_length = 1;
int segment_length_discrete_num = 2;
double steering_penalty = 1.0;
double steering_change_penalty = 1.05;
double reversing_penalty = 3.0;
double shot_distance = 1;
int grid_size_phi = 72;
/**********全局路径参数************/
GnssProcess gnss_path;
bool use_lla = true;
nav_msgs::Path global_path;//轨迹队列
geometry_msgs::PoseStamped global_path_pose;
fast_lio::Position_state position_state;
GnssProcess gnss_get_enu;
bool path_dowm_init = false;
bool gnss_init = false;
//定位回调信息
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
//读取全局轨迹并发布
void readcsv_to_path(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::strerror(errno) << std::endl;
        return;
    }
    std::string line;
    int count = 0;
    double value1, value2;
    // global_path_pose.header.frame_id = "camera_init";
    //读取文件
    while (std::getline(file, line)){
        std::string cell;
        std::stringstream ss(line);
        // 读取第一个值
        std::getline(ss, cell, ',');
        std::stringstream ss1(cell);
        ss1 >> value1;
        // 读取第二个值
        std::getline(ss, cell, ',');
        std::stringstream ss2(cell);
        ss2 >> value2;
        if (use_lla) {
            if (gnss_init == true) {
                gnss_path.update_xyz(value2, value1, 0);
                global_path_pose.pose.position.x = gnss_path.local_E;
                global_path_pose.pose.position.y = gnss_path.local_N;
                }
        } 
        else 
        {
            global_path_pose.pose.position.x = value1;
            global_path_pose.pose.position.y = value2;

        }
        if(count % 100 == 0){
            global_path.poses.push_back(global_path_pose);
        }
        count++;
        value1 = 0;
        value2 = 0;
     }
    file.close();
}
//获取目标节点(局部坐标系)
Eigen::Vector3d get_goal_state(fast_lio::Position_state& position_state_msg_in,
                                nav_msgs::Path& global_path_in)
{
    Eigen::Vector3d goal_state;
    int idx = 0;
    double goal_angle;
    double pre_angle;
    double dist[global_path_in.poses.size()];
    double pos[1][2];
    pos[0][0] = position_state_msg_in.Positon_x;
    pos[0][1] = position_state_msg_in.Positon_y;
    dist[0] = sqrt(pow(global_path_in.poses[0].pose.position.x-pos[0][0],2)+pow(global_path_in.poses[0].pose.position.y-pos[0][1],2));
    double min = dist[0];
    for(int i = 1; i< global_path_in.poses.size(); i++)
    {
        //找到距离最近的点及标签
        dist[i] = sqrt(pow(global_path_in.poses[i].pose.position.x-pos[0][0],2)+pow(global_path_in.poses[i].pose.position.y-pos[0][1],2));
        if(dist[i] < min)
        {
            min = dist[i];
        }
    }
    for(int i = 0; i < global_path_in.poses.size(); i++)
    {
        if(dist[i] == min)
        {
            idx = i;
            break;
        }
    }
    double l_step = 0;
    double ld = 12;
    while(l_step < ld & idx < global_path_in.poses.size())
    {
        l_step = l_step + sqrt(pow(global_path_in.poses[idx].pose.position.x-global_path_in.poses[idx+1].pose.position.x,2)
                                +pow(global_path_in.poses[idx].pose.position.y-global_path_in.poses[idx+1].pose.position.y,2));
        idx = idx + 1;
    }
    if (use_lla == true)
    {
        pre_angle = atan2((global_path_in.poses[idx].pose.position.y-pos[0][1]),(global_path_in.poses[idx].pose.position.x-pos[0][0])) - position_state_msg_in.rot_z + 3.1415/2 ;
        goal_angle = atan2((global_path_in.poses[idx].pose.position.y - global_path_in.poses[idx-2].pose.position.y),
                                (global_path_in.poses[idx].pose.position.x - global_path_in.poses[idx-2].pose.position.x))-position_state_msg_in.rot_z + 3.1415/2 ;
    }
    else{
        pre_angle = atan2((global_path_in.poses[idx].pose.position.y-pos[0][1]),(global_path_in.poses[idx].pose.position.x-pos[0][0]))-position_state_msg_in.rot_z ;
        goal_angle = atan2((global_path_in.poses[idx].pose.position.y - global_path_in.poses[idx-2].pose.position.y),
                                (global_path_in.poses[idx].pose.position.x - global_path_in.poses[idx-2].pose.position.x))-position_state_msg_in.rot_z;
    }
    goal_state[1] = - dist[idx]*sin(pre_angle);
    goal_state[0] = -dist[idx]*cos(pre_angle);
    if (goal_angle > 3.1415/2)goal_angle = goal_angle - 3.1415;
    if (goal_angle < -3.1415/2)goal_angle = goal_angle + 3.1415;
    goal_state[2] = goal_angle;
    return goal_state;
}
//局部地图回调函数
void detect_object_cbk(const autoware_msgs::DetectedObjectArray::ConstPtr &object_array_in)
{
    local_map.header.frame_id = "body";
    local_map.info.resolution = grid_resolution;
    local_map.info.width = grid_width;
    local_map.info.height = grid_height;
    local_map.info.origin.position.x = grid_origin_x;
    local_map.info.origin.position.y = grid_origin_y;
    local_map.info.origin.orientation.w = 1.0; 
    local_map.data.resize(grid_width * grid_height, 0);
    for (const auto &object :object_array_in->objects)
    {
        for (double xi = object.pose.position.x - object.dimensions.x/2; xi < object.pose.position.x + object.dimensions.x/2; xi += grid_resolution)
        {
            for (double yi = object.pose.position.y - object.dimensions.y/2;yi < object.pose.position.y + object.dimensions.y/2; yi += grid_resolution)
            {
                local_map.data[int((yi - grid_origin_y) / grid_resolution) * grid_width + int((xi - grid_origin_x) / grid_resolution)] = 100;
            }
        }
    }
}
//轨迹发布函数
void PublishPath(const std::vector<Eigen::Vector3d> &path, ros::Publisher& pub)
{
    nav_msgs::Path local_path;
    geometry_msgs::PoseStamped pose_stamped;
    for(const auto &pose: path){
        pose_stamped.pose.position.z = 0.0;
        double theta = 0;
        theta =  position_state.rot_z - 3.1415/2;
        Eigen::Matrix2d R;// 创建旋转矩阵
        Eigen::Vector2d po;
        R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);
            po[0] =-pose.x() ;
            po[1] = -pose.y() ;
        pose_stamped.pose.position.x = ((R * po)[0] + position_state.Positon_x);
        pose_stamped.pose.position.y = (R * po)[1] + position_state.Positon_y;
        
        local_path.poses.emplace_back(pose_stamped);
    }
    local_path.header.frame_id = "camera_init";
    //nav_path.header.stamp = timestamp_;
    pub.publish(local_path);
}
//搜索树发布
void PublishSearchedTree(const std::vector<Eigen::Vector4d> &searched_tree,ros::Publisher& pub) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "body";
    // tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;
    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;
    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;
    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }
    pub.publish(tree_list);
}
//主函数
int main(int argc,char **argv){
    ros::init(argc, argv,"astar");
    ros::NodeHandle nh;
    int kkk = 0;
    //初始化搜索器
    HybridAStar_searcher H_Astar_searcher(
                                    steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
                                    steering_penalty, reversing_penalty, steering_change_penalty, shot_distance, grid_size_phi);
    //障碍物检测回调
    ros::Subscriber object_sub = nh.subscribe<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects",1,detect_object_cbk);
    // //实例化position_state订阅者对象
    ros::Subscriber sub = nh.subscribe<fast_lio::Position_state>("/state_to_control",1,position_state_cbk);
    //局部轨迹的发布者
    ros::Publisher local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path",1);
    //环境地图发布者
    ros::Publisher local_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("local_map",1);
    //树发布者
    ros::Publisher tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    //全局轨迹发布
    ros::Publisher global_path_pub = nh.advertise<nav_msgs::Path>("global_path",1);
    //全局轨迹
    global_path.header.frame_id = "camera_init";
    std::string filename;
    if(use_lla)filename = "/home/kim/Documents/yqb/control/src/path/lla_path.txt";
    else filename = "/home/kim/Documents/yqb/control/src/path/easy_path.txt"; 
    ros::Rate rate(1);
    while (ros::ok()){
        ros::spinOnce();
        //获取全局轨迹
        if (use_lla == true && gnss_init == false)continue;
        if (path_dowm_init == false){
            readcsv_to_path(filename);
            path_dowm_init = true;
        }
        ROS_INFO("1");
        global_path_pub.publish(global_path);
        local_map_pub_.publish(local_map);
        ROS_INFO("2");     
        if (!local_map.data.empty()){
            //初始化地图参数
            ROS_INFO("3");
            H_Astar_searcher.MapInit(local_map.info.origin.position.x,
                            local_map.info.width * local_map.info.resolution + local_map.info.origin.position.x,
                            local_map.info.origin.position.y,
                            local_map.info.height * local_map.info.resolution + local_map.info.origin.position.y,
                            1.0,
                            local_map.info.resolution);
            unsigned int map_w = std::floor(local_map.info.width);
            unsigned int map_h = std::floor(local_map.info.height);
            ddd = 0;
            ROS_INFO("4");
            for (unsigned int w = 0; w < map_w; w++){
                for (unsigned int h = 0; h < map_h; h++){
                        H_Astar_searcher.map_data_[h * local_map.info.width + w] = local_map.data[h * local_map.info.width + w]/100;
                        ddd = ddd + 1;
                    }
                }
            //设置地图起点终点
            Eigen::Vector3d start_state = Eigen::Vector3d(0, 0, 0);
            ROS_INFO("5");
            //搜索路径
            if (H_Astar_searcher.Search(start_state, get_goal_state(position_state,global_path))){
                auto path = H_Astar_searcher.GetPath();
                PublishPath(path, local_path_pub_);
                PublishSearchedTree(H_Astar_searcher.GetSearchedTree(),tree_pub_);
            }
        }
        ROS_INFO("6");
        local_map.data.clear();
        H_Astar_searcher.Reset();
        rate.sleep();
    }
    return 0;
}