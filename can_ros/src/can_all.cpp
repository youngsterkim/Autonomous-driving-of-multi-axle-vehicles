#include"ros/ros.h"
#include<canlib.h>
#include<stdio.h>
#include"std_msgs/Float64.h"
#include"can_ros/read_can.h"
#include"can_ros/send_can.h"
#include "Position_state.h"
canHandle hnd;
canStatus stat;
canStatus stat1;
canStatus stat2;
canStatus stat3;
canStatus stat4;
canStatus stat5;
canStatus stat6;
canStatus stat7;
canStatus stat8;
long id;
//用于控制电机启动
bool speed_motor_control = false;
bool steering_motor_control = false;
int steering_motor_speed = 20;
//轮毂电机enable与mod之间的间隔时间
ros::Duration duration(1.0);
    int send_steering_l1 = 300;
    int send_steering_r1 = 300;
    int send_steering_l2 = 300;
    int send_steering_r2 = 300;
    int send_steering_l3 = 300;
    int send_steering_r3 = 300;
    int send_throttle_l = 0;
    int send_throttle_r = 0;
//创建数组用于发布和接收底盘数据
unsigned char ReadData_1[8] = {0,0,0,0,0,0,0,0};//接收一二三四轮的转角数据
unsigned char ReadData_2[8] = {0,0,0,0,0,0,0,0};//接收五六轮的转角和一二轮转速的数据
unsigned char ReadData_3[8] = {0,0,0,0,0,0,0,0};//接收三四五六轮转速的数据
unsigned char ReadData_4[8] = {0,0,0,0,0,0,0,0};//接收手机发送的数据
unsigned char SendData_1[8] = {0,0,0,0,0,0,0,0};//发送车轮一二三四轮转角的数据
unsigned char SendData_2[8] = {0,0,0,0,0,0,0,0};//发送车轮五六轮的转角数据和五六电机转速数据
unsigned char SendData_3[8] = {0,0,0,0,0,0,0,0};//发送ax\ay\wz\vx
unsigned char SendData_4[8] = {0,0,0,0,0,0,0,0};//beta\X\Y坐标\航向角
unsigned char SendData_5[8] = {0,0,0,0,0,0,0,0};//转向电机速度\轮毂电机enable\轮毂电机mode


unsigned int dlc;
unsigned int flag;
unsigned long t;

fast_lio::Position_state position_state;
ros::Publisher can_pub;//创建发布者
can_ros::read_can read_can;//创建底盘消息格式
//自检
void Check(const char* id,canStatus stat)
{
    if(stat !=canOK)
    {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat,buf,sizeof(buf));
        printf("%s:fail,stat=%d (%s)\n",id,(int)stat,buf);
    }
}
//接收来自上层的车辆状态信息(发送给控制器)(个别情况使用)
void state_data(const fast_lio::Position_state &position_state_in)
{
    position_state = position_state_in;

}
//发送给底盘消息，转化赋值给数组，再给底盘接收
void send_function(const can_ros::send_can& send_msg)
{
        send_steering_l1=send_msg.send_steering_l1*10+300;
        send_steering_r1=send_msg.send_steering_r1*10+300;
        send_steering_l2=send_msg.send_steering_l2*10+300;
        send_steering_r2=send_msg.send_steering_r2*10+300;
        send_steering_l3=send_msg.send_steering_l3*10+300;
        send_steering_r3=send_msg.send_steering_r3*10+300;
        send_throttle_l=send_msg.send_throttle_l;
        send_throttle_r=send_msg.send_throttle_r;
        ROS_INFO("speed_l = %d",send_throttle_l);
        ROS_INFO("speed_r = %d",send_throttle_r);

       //限制-30转角
       int xianzhijiaodu = 30;
       if(send_steering_l1<300-xianzhijiaodu*10)
       {
           send_steering_l1=300-xianzhijiaodu*10;
       }
       if(send_steering_r1<300-xianzhijiaodu*10)
       {
           send_steering_r1=300-xianzhijiaodu*10;
       }
       if(send_steering_l2<300-xianzhijiaodu*10)
       {
           send_steering_l2=300-xianzhijiaodu*10;
       }

       if(send_steering_r2<300-xianzhijiaodu*10)
       {
           send_steering_r2=300-xianzhijiaodu*10;
       }

       if(send_steering_l3<300-xianzhijiaodu*10)
       {
           send_steering_l3=300-xianzhijiaodu*10;
       }

       if(send_steering_r3<300-xianzhijiaodu*10)
       {
           send_steering_r3=300-xianzhijiaodu*10;
       }
       //限制30度转角
       if(send_steering_l1>300+xianzhijiaodu*10)
       {
           send_steering_l1=300+xianzhijiaodu*10;
       }
       if(send_steering_r1>300+xianzhijiaodu*10)
       {
           send_steering_r1=300+xianzhijiaodu*10;
       }

       if(send_steering_l2>300+xianzhijiaodu*10)
       {
           send_steering_l2=300+xianzhijiaodu*10;
       }

       if(send_steering_r2>300+xianzhijiaodu*10)
       {
           send_steering_r2=300+xianzhijiaodu*10;
       }

       if(send_steering_l3>300+xianzhijiaodu*10)
       {
           send_steering_l3=300+xianzhijiaodu*10;
       }

       if(send_steering_r3>300+xianzhijiaodu*10)
       {
           send_steering_r3=300+xianzhijiaodu*10;
       }
       //限制转速
    //    if (send_throttle_r > 20)
    //    {
    //     send_throttle_r = 20;
    //    }
    //     if (send_throttle_l > 20)
    //    {
    //     send_throttle_l = 20;
    //    }

}
int main(int argc,char **argv)
{   
    
    ros::init(argc,argv,"can_ros");
    ros::NodeHandle can_to_ros;
    canInitializeLibrary();//初始化程序
    hnd = canOpenChannel(0,canOPEN_EXCLUSIVE);//开启通道
    if(hnd != canOK) //判断can是否错误
    {
        char msg[64];
        canGetErrorText((canStatus)hnd,msg,sizeof(msg));//解释错误代码信息
        fprintf(stderr,"canopenchannel fialied(%s)\n",msg);
        exit(1);
    }
    canSetBusParams(hnd,canBITRATE_500K,0,0,0,0,0);//设定通道相关参数
    stat = canBusOn(hnd);
    Check("canBusOn",stat);

    can_pub = can_to_ros.advertise<can_ros::read_can>("/can_data",1);//发布消息
    ros::Subscriber subtt = can_to_ros.subscribe("/decision/steering_angle",1,send_function);//订阅上层消息
    ros::Subscriber subss = can_to_ros.subscribe("/state_to_control",1,state_data);//订阅上层消息
    ros::Rate r(10);
    while(ros::ok())
    {   
        
        ros::spinOnce();
        
        if (stat == canOK)
        {
            std::cout<<"init_canOK"<<std::endl;
            std::cout<<std::endl;
            canFlushTransmitQueue(hnd);

            if (send_steering_r1 != 300 && steering_motor_control == false ){
                SendData_5[0] = (unsigned char)(steering_motor_speed );
                stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
                steering_motor_control = true;
                duration.sleep();
                ROS_WARN("steering_motor_control is True! speed is %d",steering_motor_speed);
            }

            if (send_throttle_r != 0 && speed_motor_control == false){
                    SendData_5[1] = (unsigned char)(1);
                    ROS_WARN("speed_motor_control is True! enable Wait 3s!");
                    stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
                    duration.sleep();
                    SendData_5[2] = (unsigned char)(1);
                    stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
                    speed_motor_control = true;
                    ROS_WARN(" MOTOR_Mod is OK!");
                    duration.sleep();
            }
            if (stat<0) printf("canWrite failed,stat = %d\n",stat);
        }  
        //读取can信号
        stat2 = canReadSpecificSkip(hnd,0x0012L,ReadData_1,&dlc,&flag,&t);
        stat3 = canReadSpecificSkip(hnd,0x0013L,ReadData_2,&dlc,&flag,&t);
        stat4 = canReadSpecificSkip(hnd,0x0015L,ReadData_3,&dlc,&flag,&t);
        stat5 = canReadSpecificSkip(hnd,0x008L,ReadData_4,&dlc,&flag,&t);
        
        Check("canReadSpecificSkip2",stat2);
        Check("canReadSpecificSkip3",stat3);
        Check("canReadSpecificSkip4",stat4);
       
        read_can.read_steering_l1=float(ReadData_1[0]*256+ReadData_1[1]-3000)/100;
        read_can.read_steering_r1=float(ReadData_1[2]*256+ReadData_1[3]-3000)/100;
        read_can.read_steering_l2=float(ReadData_1[4]*256+ReadData_1[5]-3000)/100;
        read_can.read_steering_r2=float(ReadData_1[6]*256+ReadData_1[7]-3000)/100;
        read_can.read_steering_l3=float(ReadData_2[0]*256+ReadData_2[1]-3000)/100;
        read_can.read_steering_r3=float(ReadData_2[2]*256+ReadData_2[3]-3000)/100;
        read_can.read_speed_l1=float(ReadData_2[4]*256+ReadData_2[5]-30000)/1000;
        read_can.read_speed_r1=float(ReadData_2[6]*256+ReadData_2[7]-30000)/1000;
        read_can.read_speed_r2=float(ReadData_3[0]*256+ReadData_3[1]-30000)/1000;//这里故意r2l2对调
        read_can.read_speed_l2=float(ReadData_3[2]*256+ReadData_3[3]-30000)/1000;
        read_can.read_speed_l3=float(ReadData_3[4]*256+ReadData_3[5]-30000)/1000;
        read_can.read_speed_r3=float(ReadData_3[6]*256+ReadData_3[7]-30000)/1000;
        read_can.read_phone_run=(float)ReadData_4[0];
        read_can.read_phone_gears=(float)ReadData_4[1];
        read_can.read_phone_steering=(float)ReadData_4[2];
        read_can.read_phone_modes=(float)ReadData_4[3];
        read_can.header.stamp = ros::Time::now();//打上时间戳
        can_pub.publish(read_can);//发布can信号

/*下面是发送信号给控制器（特殊情况需要）*/
        int send_ax = position_state.state_wx*1000+10000;
        int send_ay = position_state.state_wy*1000+10000;
        int send_vx = position_state.state_vx*1000+10000;
        int send_wz = position_state.state_wz*1000+10000;
        int send_beta = position_state.beta*100+10000;
        int send_x = position_state.Positon_x*100+10000;
        int send_y = position_state.Positon_y*100+10000;
        int send_rotz = position_state.rot_z *100+10000;
        SendData_3[0]=(unsigned char)(send_ax/256);
        SendData_3[1]=(unsigned char)(send_ax%256);
        SendData_3[2]=(unsigned char)(send_ay/256);
        SendData_3[3]=(unsigned char)(send_ay%256);
        SendData_3[4]=(unsigned char)(send_wz/256);
        SendData_3[5]=(unsigned char)(send_wz%256);
        SendData_3[6]=(unsigned char)(send_vx/256);
        SendData_3[7]=(unsigned char)(send_vx%256);
        SendData_4[0]=(unsigned char)(send_beta/256);
        SendData_4[1]=(unsigned char)(send_beta%256);
        SendData_4[2]=(unsigned char)(send_x/256);
        SendData_4[3]=(unsigned char)(send_x%256);
        SendData_4[4]=(unsigned char)(send_y/256);
        SendData_4[5]=(unsigned char)(send_y%256);
        SendData_4[6]=(unsigned char)(send_rotz/256);
        SendData_4[7]=(unsigned char)(send_rotz%256);
/*********************************）*/
        SendData_1[0]=(unsigned char)(send_steering_l1/256);
        SendData_1[1]=(unsigned char)(send_steering_l1%256);
        SendData_1[2]=(unsigned char)(send_steering_r1/256);
        SendData_1[3]=(unsigned char)(send_steering_r1%256);
        SendData_1[4]=(unsigned char)(send_steering_l2/256);
        SendData_1[5]=(unsigned char)(send_steering_l2%256);
        SendData_1[6]=(unsigned char)(send_steering_r2/256);
        SendData_1[7]=(unsigned char)(send_steering_r2%256);
        SendData_2[0]=(unsigned char)(send_steering_l3/256);
        SendData_2[1]=(unsigned char)(send_steering_l3%256);
        SendData_2[2]=(unsigned char)(send_steering_r3/256);
        SendData_2[3]=(unsigned char)(send_steering_r3%256);
        SendData_2[4]=(unsigned char)(send_throttle_l/256);
        SendData_2[5]=(unsigned char)(send_throttle_l%256);
        SendData_2[6]=(unsigned char)(send_throttle_r/256);
        SendData_2[7]=(unsigned char)(send_throttle_r%256);
        ROS_INFO("%f",read_can.read_steering_l1);
        ROS_INFO("%f",read_can.read_steering_r1);
        ROS_INFO("%f",read_can.read_steering_l2);
        ROS_INFO("%f",read_can.read_steering_r2);
        ROS_INFO("%f",read_can.read_steering_l3);
        ROS_INFO("%f",read_can.read_steering_r3);
        ROS_INFO("%f",read_can.read_speed_l1);
        ROS_INFO("%f",read_can.read_speed_r1);
        ROS_INFO("%f",read_can.read_speed_l2);
        ROS_INFO("%f",read_can.read_speed_r2);
        ROS_INFO("%f",read_can.read_speed_l3);
        ROS_INFO("%f",read_can.read_speed_r3);
        //写入信号
        if (stat == canOK)
        {
            std::cout<<"canOK"<<std::endl;
            std::cout<<std::endl;
            canFlushTransmitQueue(hnd);
            stat = canWrite(hnd,0x7,SendData_1,8,canMSG_STD);//写入7通道
            stat1 = canWrite(hnd,0x16,SendData_2,8,canMSG_STD);//写入16通道
            stat6 = canWrite(hnd,0x1,SendData_3,8,canMSG_STD);//写入1通道
            stat7 = canWrite(hnd,0x2,SendData_4,8,canMSG_STD);//写入2通道
            stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
            if (stat<0) printf("canWrite failed,stat = %d\n",stat);
        }  
        r.sleep();
    }
    if (stat == canOK)
    {
        std::cout<<"over_canOK"<<std::endl;
        std::cout<<std::endl;
        canFlushTransmitQueue(hnd);
        //程序结束关闭电机信号及所有控制参数置零
        SendData_1[0]=(unsigned char)(300/256);
        SendData_1[1]=(unsigned char)(300%256);
        SendData_1[2]=(unsigned char)(300/256);
        SendData_1[3]=(unsigned char)(300%256);
        SendData_1[4]=(unsigned char)(300/256);
        SendData_1[5]=(unsigned char)(300%256);
        SendData_1[6]=(unsigned char)(300/256);
        SendData_1[7]=(unsigned char)(300%256);
        SendData_2[0]=(unsigned char)(300/256);
        SendData_2[1]=(unsigned char)(300%256);
        SendData_2[2]=(unsigned char)(300/256);
        SendData_2[3]=(unsigned char)(300%256);
        SendData_2[4]=(unsigned char)(0/256);
        SendData_2[5]=(unsigned char)(0%256);
        SendData_2[6]=(unsigned char)(0/256);
        SendData_2[7]=(unsigned char)(0%256);
        stat = canWrite(hnd,0x7,SendData_1,8,canMSG_STD);//写入7通道
        stat1 = canWrite(hnd,0x16,SendData_2,8,canMSG_STD);//写入16通道
        //等待转角归0
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        duration.sleep();
        //关闭转向电机开关
        SendData_5[0] = (unsigned char)(0);
        ROS_WARN("steering_motor_control is off");
        //关闭轮毂电机模块
        SendData_5[2] = (unsigned char)(0);
        ROS_WARN(" MOTOR_Mod is OFF!");
        stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
        duration.sleep();//等待关闭mod
        SendData_5[1] = (unsigned char)(0);
        ROS_WARN("speed_motor_control Enable is OFF!");
        stat8 = canWrite(hnd,0x3,SendData_5,8,canMSG_STD);//写入3通道
        duration.sleep();
        if (stat<0) printf("canWrite failed,stat = %d\n",stat);
    }  
    canFlushReceiveQueue(hnd);
    canBusOff(hnd);
    canClose(hnd);
    return 0;
}
