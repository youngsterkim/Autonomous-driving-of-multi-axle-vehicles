#include<Eigen/Core>
#include<Eigen/Eigen>
#include<stdio.h>
#include<cmath>
#pragma once

struct StateNode{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //状态点所属列
    enum NODE_STATUS{
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };
    //状态点方向
    enum DIRECTION{
        FORWARD = 0, BACKWARD = 1, NO = 3
    };
    StateNode() = delete;
    explicit StateNode(const Eigen::Vector3i &grid_index){
        node_status_ = NOT_VISITED;
        grid_index_  = grid_index;
        parent_node_ = nullptr;
    }
    void Reset(){
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }
    NODE_STATUS node_status_;
    DIRECTION direction_{};
    Eigen::Vector3d state_;
    Eigen::Vector3i grid_index_;
    double g_cost_{},f_cost_{};
    int steering_grade_{};
    StateNode *parent_node_;//父节点
    typedef StateNode *Ptr;
    std::vector<Eigen::Vector3d> intermediate_states_;//存储到达该节点过程中的状态
};