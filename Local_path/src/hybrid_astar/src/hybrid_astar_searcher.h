#include<stdio.h>
#include<cmath>
#include "state_node.hpp"
#include<Eigen/Core>
#include<Eigen/Eigen>
#include "rs_path.h"
#include<map>
#include<memory>
#include <ros/ros.h>

class HybridAStar_searcher
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //构造初始化
    HybridAStar_searcher(double steering_angle, int steering_angle_discrete_num, double segment_length,
                        int segment_length_discrete_num, double wheel_base, double steering_penalty,
                        double reversing_penalty, double steering_change_penalty, double shot_distance, int grid_size_phi = 72);
    //析构
    ~HybridAStar_searcher();
    //地图参数初始化
    void MapInit(double x_lower, double x_upper, double y_lower, double y_upper,
                    double state_grid_resolution, double map_grid_resolution);
    //搜索函数
    bool Search(const Eigen::Vector3d &state, const Eigen::Vector3d &goal_state);
    //获得搜索树
    std::vector<Eigen::Vector4d> GetSearchedTree();
    //h获得路径
    std::vector<Eigen::Vector3d> GetPath();
    //坐标转地图索引
    Eigen::Vector2i Coordinate2MapGridIndex(const Eigen::Vector2d &pt);
    //设置车辆参数
    void SetVehicleShape(double length, double width, double rear_axle_dist);
    //重置类
    void Reset();
    uint8_t *map_data_ = nullptr;
private:
    //障碍判断

    inline bool HasObstacle(int grid_index_x, int grid_index_y);
    inline bool HasObstacle(const Eigen::Vector2i &grid_index);
    //检查是否碰撞
    bool CheckCollision(const double &x, const double &y, const double &theta);
    //执行射线检测
    inline bool LineCheck(double x0, double y0, double x1, double y1);
    //延展分析
    bool AnalyticExpansions(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &goal_node_ptr, double &length);
    //计算G值
    inline double ComputeG(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &neighbor_node_ptr);
    //计算H值
    inline double ComputeH(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &terminal_node_ptr);
    //状态转为网格
    inline Eigen::Vector3i State2Index(const Eigen::Vector3d &state);
    //将网格转为坐标
    inline Eigen::Vector2d MapGridIndex2Coordinate(const Eigen::Vector2i &grid_index);
    //获取邻节点
    void GetNeighborNodes(const StateNode::Ptr &curr_node_ptr, std::vector<StateNode::Ptr> &neighbor_nodes);
    //动态车辆模型
    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta);
    //角度调整到-pi到pi
    inline double Mod2Pi(const double &x);
    //检查是否超出边界
    bool BeyondBoundary(const Eigen::Vector2d &pt);
    void ReleaseMemory();
    
    //类中地图
    double STATE_GRID_RESOLUTION_{},MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{},STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};
    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};
    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr ***state_node_map_ = nullptr;
    std::multimap<double, StateNode::Ptr> openset_;
    double wheel_base_; //轴距
    double segment_length_;//路径段长度
    double move_step_size_;//移动步长
    double steering_radian_step_size_;//转向角步长
    double steering_radian_; //弧度转角
    double tie_breaker_; //平局打破系数
    double shot_distance_;//射线检测的距离
    int segment_length_discrete_num_;//路径段长度离散化数量
    int steering_discrete_num_;//转向角离散化数量
    double steering_penalty_;//转向惩罚系数
    double reversing_penalty_;//倒车惩罚系数
    double steering_change_penalty_;//转向变化惩罚系数
    double path_length_ = 0.0;// 路径长度
    std::shared_ptr<RSPath> rs_path_ptr_;//对象指针
    Eigen::VectorXd vehicle_shape_; //车辆形状
    Eigen::MatrixXd vehicle_shape_discrete_;//离散化的车辆形状。
    // debug
    double check_collision_use_time = 0.0;//检测碰撞所花费的时间
    int num_check_collision = 0;// 检测碰撞的次数
    int visited_node_number_ = 0;//访问过的节点数
};