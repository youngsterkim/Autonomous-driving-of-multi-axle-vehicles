#include "hybrid_astar_searcher.h"

//构造
HybridAStar_searcher::HybridAStar_searcher(double steering_angle, int steering_angle_discrete_num, double segment_length,
                                            int segment_length_discrete_num, double wheel_base, double steering_penalty,
                                        double reversing_penalty, double steering_change_penalty, double shot_distance, int grid_size_phi){
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI/180.0;
    steering_discrete_num_ = segment_length_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;
    //轨迹传参转向半径
    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;
    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI /180.0;
}
//析构
HybridAStar_searcher::~HybridAStar_searcher(){
    ReleaseMemory();
}
//初始化
void HybridAStar_searcher::MapInit(double x_lower, double x_upper, double y_lower, double y_upper,
                                    double state_grid_resolution, double map_grid_resolution){
    SetVehicleShape(3.0, 1.5, 2.0);
    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;
    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);
    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);
    //如果存在地图
    if (map_data_){
        delete[] map_data_;
        map_data_ = nullptr;
    }

    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];
    //如果存在状态,清理状态点集
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }
        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }
    //重新分配节点地图
    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}
//用于检查两点之间是否存在障碍物（Bresenham算法）
inline bool HybridAStar_searcher::LineCheck(double x0, double y0, double x1, double y1) {
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));//确定主要迭代轴
    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step; 
    auto yk = y0;
    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }
    auto N = static_cast<unsigned int>(x1 - x0);// 计算需要遍历的点数
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            if (HasObstacle(Eigen::Vector2i(yk, x0 + i * 1.0))// 检查当前点是否可通
                || BeyondBoundary(Eigen::Vector2d(yk * MAP_GRID_RESOLUTION_ + map_x_lower_, (x0 + i) * MAP_GRID_RESOLUTION_+ map_y_lower_))) {
                return false;
            }
        } else {
            if (HasObstacle(Eigen::Vector2i(x0 + i * 1.0, yk))// 检查当前点是否有可通
                || BeyondBoundary(Eigen::Vector2d((x0 + i) * MAP_GRID_RESOLUTION_+ map_x_lower_,yk * MAP_GRID_RESOLUTION_ + map_y_lower_))
                    ) {
                return false;
            }
        }
        error += delta_error;
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }
    return true;
}
//车辆碰撞检测
bool HybridAStar_searcher::CheckCollision(const double &x, const double &y, const double &theta) {
    Eigen::Matrix2d R;// 创建旋转矩阵
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);
    Eigen::MatrixXd transformed_vehicle_shape;// 创建变换后的车辆形状矩阵
    transformed_vehicle_shape.resize(8, 1); // 设置矩阵大小为 8 行 1 列
    for (unsigned int i = 0; i < 4u; ++i) { // 遍历车辆四个角点
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Eigen::Vector2d(x, y);
    }
    //ROS_INFO("x= %f,y =%f",x,y);
    Eigen::Vector2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0));
    Eigen::Vector2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0));
    Eigen::Vector2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0));
    Eigen::Vector2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0));
    double y1, y0, x1, x0; // 定义变量用于存储坐标
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());
    if (!LineCheck(x1, y1, x0, y0)) {// 使用 LineCheck 方法检查是否有障碍物
        return false;// 如果有障碍物，则返回 false
    }
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());
    if (!LineCheck(x1, y1, x0, y0)) {// 使用 LineCheck 方法检查是否有障碍物
        return false; // 如果有障碍物，则返回 false
    }
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());
    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());
    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }
    num_check_collision++; // 增加碰撞检查次数计数器
    return true;// 如果没有遇到障碍物，则返回 true
}
//检测是否是障碍点（索引）
bool HybridAStar_searcher::HasObstacle(const int grid_index_x, const int grid_index_y) {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_ 
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}
//检测是否是障碍点（向量）
bool HybridAStar_searcher::HasObstacle(const Eigen::Vector2i &grid_index){
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}
//设置车辆参数
void HybridAStar_searcher::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8); // 调整车辆形状矩阵大小为 8 行 1 列
    vehicle_shape_.block<2, 1>(0, 0) = Eigen::Vector2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Eigen::Vector2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Eigen::Vector2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Eigen::Vector2d(-rear_axle_dist, -width / 2);
    const double step_size = move_step_size_;// 获取移动步长
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);
    const Eigen::Vector2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) { 
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }
    const Eigen::Vector2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0) 
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) { 
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}
//二维转地图
Eigen::Vector2d HybridAStar_searcher::MapGridIndex2Coordinate(const Eigen::Vector2i &grid_index)  {
    Eigen::Vector2d pt; 
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt; 
}
//将状态向量转换为网格索引。
Eigen::Vector3i HybridAStar_searcher::State2Index(const Eigen::Vector3d &state)  {
    Eigen::Vector3i index;
    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
    return index;
}
//实际坐标转换为地图网格索引
Eigen::Vector2i HybridAStar_searcher::Coordinate2MapGridIndex(const Eigen::Vector2d &pt)  {
    Eigen::Vector2i grid_index;
    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}
//搜索邻近状态点
void HybridAStar_searcher::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {
    
    neighbor_nodes.clear();
    
    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) { 
        std::vector<Eigen::Vector3d> intermediate_state;
        bool has_obstacle = false;
        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();
        const double phi = i * steering_radian_step_size_;
        // 前向状态遍历
        
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(move_step_size_, phi, x, y, theta);// 更新位置和朝向
            intermediate_state.emplace_back(Eigen::Vector3d(x, y, theta));
            if (!CheckCollision(x, y, theta)) {//车辆碰撞检测
                has_obstacle = true;
                break;
            }
        }
        Eigen::Vector3i grid_index = State2Index(intermediate_state.back());
        
        

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {// 检查是否超出边界且无碰撞，设置状态点并加入邻近点列表
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state; 
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i; 
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
           
        }
        // 后向遍历
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta); // 更新位置和朝向
            intermediate_state.emplace_back(Eigen::Vector3d(x, y, theta));
            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                
                break;
            }
        }
        
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {// 检查是否超出边界且无碰撞，设置状态点并加入邻近点列表
            
            
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD; 
            neighbor_nodes.push_back(neighbor_backward_node_ptr); 
            
        }
    }
}
//动态模型
void HybridAStar_searcher::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta)  {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}
//确保角度在-pi到pi间
double HybridAStar_searcher::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}
//边缘检测
bool HybridAStar_searcher::BeyondBoundary(const Eigen::Vector2d &pt) {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}
// 计算启发式函数 H 值
double HybridAStar_searcher::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    //曼哈顿距离
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();
    if (h < 3.0 * shot_distance_) {//小于一定阈值用RS曲线算法
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }
    return h;
}
//计算代价函数 G 值
double HybridAStar_searcher::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr){
    double g;// 定义代价g
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) { //前/后向代价
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {//变转向代价
            if (neighbor_node_ptr->steering_grade_ == 0) {// 转向代价
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else { 
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }
    return g;
}
//搜索函数
bool HybridAStar_searcher::Search(const Eigen::Vector3d &start_state, const Eigen::Vector3d &goal_state) {
    // 将起始状态和目标状态转换为离散的栅格索引
    const Eigen::Vector3i start_grid_index = State2Index(start_state);
    const Eigen::Vector3i goal_grid_index = State2Index(goal_state);
     //ROS_INFO("goal_has%d",HasObstacle(goal_grid_index[0], goal_grid_index[1]));
    // 初始化目标点、起点
    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;
    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;//置为openset
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    //计算初始代价、启发值、加入状态地图
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);
    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;
    // 清空开放列表，并将起始节点加入开放列表、状态点初始化
    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));
    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;
    int count = 0;
    //处理openset列表的所有元素
    ros::Time t1 = ros::Time::now();
    ROS_INFO("S1");
    while (!openset_.empty()) {
        
        //处理列表头
        current_node_ptr = openset_.begin()->second; 
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;//当前点加入到closeset
        openset_.erase(openset_.begin()); 
        ROS_INFO("S2");
        // 如果当前节点到目标节点的距离小于一定阈值，尝试执行解析扩展（AnalyticExpansions）
        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
            
            double rs_length = 0.0;
            ROS_INFO("S3");
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {// 如果解析扩展成功，标记目标节点为终端节点
                ROS_INFO("SA1");
                terminal_node_ptr_ = goal_node_ptr;
                // // 计算路径的长度
                // StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                // while (grid_node_ptr != nullptr) {
                //     ROS_INFO("S4");
                //     grid_node_ptr = grid_node_ptr->parent_node_;
                //     path_length_ = path_length_ + segment_length_;
                // }
                // // 更新路径长度，考虑解析扩展后的剩余路径
                // path_length_ = path_length_ - segment_length_ + rs_length;
                // ROS_INFO("Length = %f",path_length_);
                return true;
            }
        }
        // 获取、遍历当前节点的所有邻居节点
        ROS_INFO("S5");
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        // ROS_INFO("neighbor_nodes_ptr.size = %lu",neighbor_nodes_ptr.size());
        ROS_INFO("S6");
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            
            neighbor_node_ptr = neighbor_nodes_ptr[i];
            // 计算从当前节点到邻居节点的代价、启发
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            const Eigen::Vector3i &index = neighbor_node_ptr->grid_index_;
            ROS_INFO("S7");
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {// 如果邻居节点尚未被访问过
            neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;//计算G节点代价
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;//计算总代价f
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));//将总代价和状态节点添加到开放节点openlist
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;//加入状态地图
                
                continue;
            }
            else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {// 如果邻居节点已在开放列表中
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;//计算总代价
                ROS_INFO("S8");
                
                // 如果找到了一条更优的路径，更新父节点
                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    //更新点，重新加入到openlist
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                    
                }
                else {
                    ROS_INFO("S9");
                    delete neighbor_node_ptr;
                    
                }
                continue;
            }
            else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {// 如果邻居节点已在关闭列表中，则删除该节点
            ROS_INFO("S10");
                delete neighbor_node_ptr;
                
                continue;
            }
        }
        ROS_INFO("S11");
        count++;// 如果迭代次数超过 50000 次，认为搜索失败并返回 false
        if (count > 1000) {
            ROS_INFO("Exceeded the number of iterations, the search failed");
            return false;
        }
    }
    ROS_INFO("S12");
            ros::Time t2 = ros::Time::now();
        //ROS_WARN("false!!  spend time = %f",(t2-t1).toSec());
    return false;
}
//访问的节点树结构
std::vector<Eigen::Vector4d> HybridAStar_searcher::GetSearchedTree() {
    std::vector<Eigen::Vector4d> tree;
    Eigen::Vector4d point_pair;
    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                    continue;
                }
                //访问过的点在intermediate中
                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);
                    tree.emplace_back(point_pair);
                }
                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }
    return tree;
}
//分配动态内存
void HybridAStar_searcher::ReleaseMemory() {
    if (map_data_ != nullptr) {// 如果指向地图数据的指针 `map_data_` 不为空，则释放其分配的内存，并将其置为空指针。
        delete[] map_data_;
        map_data_ = nullptr;
    }
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr; 
        }
        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }
    terminal_node_ptr_ = nullptr;
}
//获取路径
std::vector<Eigen::Vector3d> HybridAStar_searcher::GetPath() {
    std::vector<Eigen::Vector3d> path;
    std::vector<StateNode::Ptr> temp_nodes;
    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;//取出父节点
    }
    // 由于节点是从终点到起点存储的，所以需要将 `temp_nodes` 反转，使得顺序从起点到终点
    std::reverse(temp_nodes.begin(), temp_nodes.end());

    // 遍历反转后的节点列表，将每个节点的中间状态添加到 `path` 向量中
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }

    return path;
}
//将内部数据结构恢复到初始状态、以便进行新的搜索。
void HybridAStar_searcher::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }
    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}
//RS曲线延展
bool HybridAStar_searcher::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    std::vector<Eigen::Vector3d> rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);
    for (const auto &pose: rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };
    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;
    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);
    return true;
}