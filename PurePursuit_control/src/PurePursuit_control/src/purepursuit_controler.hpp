#include"ros/ros.h"
#include<stdio.h>
#include<cmath>
#include<Eigen/Core>
#include<Eigen/Eigen>
#include "Position_state.h"
#include "Preview_error.h"
class purepursuit_controler
{
    public:
    //横向误差
    double Preview_dis_y = 0;
    //纵向误差
    double Preview_dis_x = 0;
    double Preview_dis = 0;
    //前轮转角
    double delta1 = 0;
    float L = 1.016;
    Preview_error Preview_error_;
    void purepursuit_controler_init(Preview_error& Preview_error_in);
    void purepursuit_controler_delta();
};
void purepursuit_controler::purepursuit_controler_init(Preview_error& Preview_error_in)
{
    Preview_dis_y = Preview_error_in.Preview_dis_y;
    Preview_dis_x = Preview_error_in.Preview_dis_x;
    Preview_dis = Preview_error_in.Preview_dis;
}
void purepursuit_controler::purepursuit_controler_delta()
{
    delta1 = atan2(2 * Preview_dis_y *L,Preview_dis*Preview_dis);
}