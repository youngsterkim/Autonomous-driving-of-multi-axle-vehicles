#include"ros/ros.h"
#include<stdio.h>
#include"std_msgs/Float64.h"
#include"can_ros/send_can.h"
#include <iostream>
#include<termios.h>


float L1_2 = 1.033;//第一轴到第二轴的距离
float L3_2 = 0.983;//第三轴到第二轴的距离
float L1_3 = 2.016;//第一轴到第三轴的距离
float W = 1.671;//轮距
float speed = 0;
float speed_cur = 0;
float input_steer = 0;
bool mod_lock = true;
bool mod_5 = false;
float steer_r1 = 0;
float steer_l1 = 0;
float steer_r2 = 0;
float steer_l2 = 0;
float steer_r3 = 0;
float steer_l3 = 0;
float R = 0;
int mod = 0;
float m = 375,C1=-6659*2,C2=-7280*2,C3=-8084.5*2,x1=1.031,x2=0.025,x3=0.958,L1=1.033,L2=0.983,L=2.016,u = 0;
float Kc3 = 0;
float D = 0;

ros::Duration duration(0.5);
//蟹形转向模式
void crab_steering_mode(float& input_steer_in)
{
    if(input_steer_in >= 25)
    {
        input_steer_in = 25;
    }
    if(input_steer_in <= -25)
    {
        input_steer_in = -25;
    }
    steer_r1 = input_steer_in;
    steer_l1 = input_steer_in;
    steer_r2 = input_steer_in;
    steer_l2 = input_steer_in;
    steer_r3 = input_steer_in;
    steer_l3 = input_steer_in;
}
//一二轴阿克曼
void akm_one_two_axle(float& input_steer_in)
{
    if(input_steer_in >= 15)
    {
    input_steer_in = 15;
    }
    if(input_steer_in <= -25)
    {
        input_steer_in = -25;
    }
    R = L1_3/tan(input_steer_in*3.1415/180);
    steer_r1 = input_steer_in;
    steer_l1 = atan(L1_3/(R - W))*180/3.1415;
    steer_r2 = atan(L3_2/R)*180/3.1415;
    steer_l2 = atan(L3_2/(R - W))*180/3.1415;
    steer_r3 = 0;
    steer_l3 = 0;
}
//一三轴阿克曼
void akm_one_three_axle(float& input_steer_in)
{
    if(input_steer_in >= 15)
    {
    input_steer_in = 15;
    }
    if(input_steer_in <= -25)
    {
        input_steer_in = -25;
    }
    R = L1_2/tan(input_steer_in*3.1415/180);
    steer_r1 = input_steer_in;
    steer_l1 = atan(L1_2/(R - W))*180/3.1415;
    steer_r2 = 0;
    steer_l2 = 0;
    steer_r3 = -atan(L3_2/R)*180/3.1415;
    steer_l3 = -atan(L3_2/(R- W))*180/3.1415;
}
//零质心阿克曼\输入为左轮
void akm_zero_mass(float& input_steer_in)
{
    if(input_steer_in >= 25)
    {
        input_steer_in = 25;
    }
    if(input_steer_in <= -15)
    {
        input_steer_in = -15;
    }
    Kc3=((x1*C1*L-x2*C2*L2)*m*u*u+2*(C1*C2*L1*L1+C1*C3*L*L+C2*C3*L2*L2)*x3)/(m*0*0*(x2*C2*L1+x3*C3*L)-2*(C1*C2*L1*L1+C1*C3*L*L+C2*C3*L2*L2)*x1);
    D=L/((1/Kc3)-1);
    R=(D+L1+L2)/tan(input_steer_in*3.1415/180)+W/2;
    steer_l1 = input_steer_in;
    steer_r1=atan((D+L1+L2)/(R+W/2))*180/3.1415;
    steer_l2=atan((D+L2)/(R-W/2))*180/3.1415;
    steer_r2=atan((D+L2)/(R+W/2))*180/3.1415;
    steer_l3=atan(D/(R-W/2))*180/3.1415;
    steer_r3=atan(D/(R+W/2))*180/3.1415;
}
void akm_point_delta()
{
    double x0 = 0;
    double y0 = 0;
    double xx = 0;
    double yy = 0;
    double CL1 = 1.1724, CL2 = 0.1394 ,CL3 = -0.8436;

    ROS_INFO("Please input x0 : -50 to 50 ");
    std::cin >> xx;
    while (xx > 50 || xx <-50){
        ROS_INFO("Please input x0 : -50 to 50 ");
        std::cin >> xx;
    }
    x0 = xx;
    ROS_INFO("Please input y0 : %f to 100 and %f to -100 ",2*xx,-2*xx);
    std::cin >> yy;
    if ( xx > 0){
        while (yy < 2*xx && yy > -2*xx || yy > 100 || yy < -100){
        ROS_INFO("Please input y0 : %f to 100 and %f to -100 ",2*xx,-2*xx);
        std::cin >> yy;
        }
    }
    if ( xx < 0){
        while (yy < -2*xx && yy > 2*xx || yy >100 || yy < -100){
        ROS_INFO("Please input y0 : %f to 100 and %f to -100 ",-2*xx,2*xx);
        std::cin >> yy;
    }
    }
    y0 = yy;
    ROS_INFO("x0 = %f,y0 = %f",x0,y0);
    steer_l1 = atan((CL1-x0)/y0)*180/3.1415;
    steer_r1 = atan((CL1-x0)/(y0+W))*180/3.1415;
    steer_l2 = atan((CL2-x0)/y0)*180/3.1415;
    steer_r2 = atan((CL2-x0)/(y0+W))*180/3.1415;
    steer_l3 = atan((CL3-x0)/y0)*180/3.1415;
    steer_r3 = atan((CL3-x0)/(y0+W))*180/3.1415;
    xx = 0;
    yy = 0;
    x0 = 0;
    y0 = 0;
    mod_5 = false;

}
int getch() 
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
int main(int argc,char *argv[])
{
    ros::init(argc,argv,"send");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<can_ros::send_can>("/decision/steering_angle",1000);
    can_ros::send_can msg ;
    ros::Rate r(100);
    ros::Duration(1).sleep();

    while(ros::ok())
    {   
        char key = getch();
        if(key)
        switch(key)
        {
            case 'w':
                speed += 5;
                break;
            case 's':
                speed -= 5;
                break;
            case 'a':
                input_steer += 5;
                break;
            case 'd':
                input_steer -= 5;
                break;
            case '1'://一二轴akm
                mod = 1;
                mod_lock = false;
                break;
            case '2'://一三轴阿克曼
                mod = 2;
                mod_lock = false;
                break;
            case '3'://蟹形
                mod = 3;
                mod_lock = false;
                break;
            case '4'://零质心
                mod = 4;
                mod_lock = false;
                break;
            case '5'://给转向中心得到
                mod = 5;
                mod_lock = false;
                mod_5 = true;
            
                break;
            case '0'://停止key控制
                mod = 0;
                break;
        }
//速度限位
        if(speed >= 20)
        {
            speed = 20;
        }
        if(speed <= -20)
        {
            speed = -20;
        }
        speed_cur = speed;//记录键盘控制速度
        //无模式控制,直接返回
        if (mod == 0){
            input_steer = 0;
            ROS_INFO("*************");
            ROS_WARN("NOT KEY CONTROL! mod = 0");
            ROS_INFO("*************");
            speed_cur = speed; 
            while (speed != 0 && mod_lock == false){

                msg.send_steering_r1 = 0;
                msg.send_steering_l1 = 0;
                msg.send_steering_r2 = 0;
                msg.send_steering_l2 = 0;
                msg.send_steering_r3 = 0;
                msg.send_steering_l3 = 0;
                if(speed > 0){
                    msg.send_throttle_r = speed - abs(speed_cur/4);
                    msg.send_throttle_l = speed - abs(speed_cur/4);
                    speed = speed - abs(speed_cur/4);
                }
                if(speed < 0){
                    msg.send_throttle_r = -speed - abs(speed_cur/4) + 100;
                    msg.send_throttle_l = -speed - abs(speed_cur/4) + 100;
                    speed = speed + abs(speed_cur/4);
                }
                ROS_INFO("spr =%.2f",msg.send_throttle_r);
                ROS_INFO("spl =%.2f",msg.send_throttle_l);
                pub.publish(msg);
                duration.sleep();
            }
            if (speed == 0 || speed == 100 || mod_lock == true){
                    msg.send_steering_r1 = 0;
                    msg.send_steering_l1 = 0;
                    msg.send_steering_r2 = 0;
                    msg.send_steering_l2 = 0;
                    msg.send_steering_r3 = 0;
                    msg.send_steering_l3 = 0;
                    msg.send_throttle_r = 0;
                    msg.send_throttle_l = 0;
                    ROS_INFO("spr00 =%.2f",msg.send_throttle_r);
                    ROS_INFO("spl00 =%.2f",msg.send_throttle_l);
                    speed = 0;
                    mod_lock = true;
                    pub.publish(msg);
            }
            continue;
        }
        //转向模式//运行模式
        if (mod == 1){
            akm_one_two_axle(input_steer);//一二轴
        }
        if (mod == 2){
            akm_one_three_axle(input_steer);//一三轴
        }
        if (mod == 3){
        crab_steering_mode(input_steer);//蟹形
        }
        if (mod == 4){
            akm_zero_mass(input_steer);//零质心
        }
        if (mod == 5 && mod_5 == true){
            akm_point_delta();
        }

//输出转角限位
         if(steer_r1 >= 25)
        {
            steer_r1= 25;
        }
        if(steer_r1<= -25)
        {
            steer_r1 = -25;
        }
        if(steer_l1 >= 25)
        {
            steer_l1 = 25;
        }
        if(steer_l1 <= -25)
        {
            steer_l1 = -25;
        }
        if(steer_r2 >= 25)
        {
            steer_r2 = 25;
        }
        if(steer_r2 <= -25)
        {
            steer_r2 = -25;
        }
        if(steer_l2 >= 25)
        {
            steer_l2 = 25;
        }
        if(steer_l2 <= -25)
        {
            steer_l2 = -25;
        }
        if(steer_r3 >= 25)
        {
            steer_r3 = 25;
        }
        if(steer_r3 <= -25)
        {
            steer_r3 = -25;
        }
        if(steer_l3 >= 25)
        {
            steer_l3 = 25;
        }
        if(steer_l3 <= -25)
        {
            steer_l3 = -25;
        }
//发送转角与速度
            msg.send_steering_r1 = steer_r1;
            msg.send_steering_l1 = steer_l1;
            msg.send_steering_r2 = steer_r2;
            msg.send_steering_l2 = steer_l2;
            msg.send_steering_r3 = steer_r3;
            msg.send_steering_l3 = steer_l3;
            if (speed >= 0){
                msg.send_throttle_r = speed;
                msg.send_throttle_l = speed;
            }
            if (speed < 0){
                msg.send_throttle_r = -speed + 100;
                msg.send_throttle_l = -speed + 100;
            }
            ROS_INFO("*************");
            ROS_INFO("up W");
            ROS_INFO("turn left A,turn right D,mod 1234");
            ROS_INFO("any key stop");
            ROS_INFO("*************");
            ROS_INFO("r1 =%.2f",msg.send_steering_r1);
            ROS_INFO("l1= %.2f",msg.send_steering_l1);
            ROS_INFO("r2 =%.2f",msg.send_steering_r2);
            ROS_INFO("l2 =%.2f",msg.send_steering_l2);
            ROS_INFO("r3 =%.2f",msg.send_steering_r3);
            ROS_INFO("l3 =%.2f",msg.send_steering_l3);
            ROS_INFO("spr =%.2f",msg.send_throttle_r);
            ROS_INFO("spl =%.2f",msg.send_throttle_l);
            ROS_INFO("mod =%d",mod);
            pub.publish(msg);
        
        r.sleep();
    }
    return 0;
}
