#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include "ros/ros.h"
#include <modbus.h>
#include <robot_arm/dataturnto.h>
#include <robot_msgs/Joint_Cmd.h>
#include <tf/transform_listener.h>
#include <thread>
#include <pthread.h>

 
// 线程1. 读取发布线程 20hz，
// 线程2. service ,接收自动运动请求，设置写入标志

using namespace std;

class Modbus_Master
{
    private:
    // for modbus
    modbus_t *ctx;
    const string slave_ip = "192.168.1.12";
    const int slave_port = 502;
    const int slave_id = 1;
    const double valve_close_current[5] = {0,0,0,0,0};

    double ao_register[8] = {0};

    // for ros
    ros::NodeHandle nh;

    public:

    // subs
    ros::Subscriber sub_joint_cmd;
    void joint_cmd_callback(const robot_msgs::Joint_Cmd::ConstPtr msg);                           // 接收手柄指令
    robot_msgs::Joint_Cmd joint_cmd;


    // functions
    Modbus_Master();
    ~Modbus_Master(){};

    // modbus tcp init
    bool init_modbus_master();   
                      

};

bool Modbus_Master::init_modbus_master()
{
    // 1. set address and port
    ctx = modbus_new_tcp(slave_ip.data(), slave_port);              //192.168.1.200  
    modbus_set_slave(ctx,slave_id);

    if (NULL == ctx)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return false;
    }
    else
    {
        std::cout<<"设置TCP成功"<<std::endl;
    }

    // 2. 设置Debug模式
    int ret = modbus_set_debug(ctx, FALSE);
    if (-1 == ret)
    {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return false;
    }else{
        std::cout<< "设置Debug模式成功"<<std::endl;
    }

    //3. set timeout
    uint32_t old_response_to_sec;
    uint32_t old_response_to_usec;
    modbus_get_byte_timeout(ctx, &old_response_to_sec, &old_response_to_usec);

    uint32_t set_response_to_sec = 0;
    uint32_t set_response_to_usec = 3000000;
    modbus_set_response_timeout(ctx, set_response_to_sec, set_response_to_usec);

    //4. 连接Server
    ret = modbus_connect(ctx);
    if (-1 == ret)
    {
        std::cout<<"Connection failed:"<<modbus_strerror(errno)<<std::endl;
        modbus_free(ctx);
        return false;
    }
    else{
        std::cout<<"Connection Ok"<<std::endl;
    }
    return true;
};

void Modbus_Master::joint_cmd_callback(const robot_msgs::Joint_Cmd::ConstPtr msg)
{
    joint_cmd = *msg;
};

Modbus_Master::Modbus_Master()
{   
    sub_joint_cmd = nh.subscribe("/joint_cmd", 100, &Modbus_Master::joint_cmd_callback, this);

    // 初始化 modbus
    if(!init_modbus_master()){
        ROS_FATAL("init_modbus_master failed");
        exit(0);
    }

    ros::Rate loop(2);
    while(ros::ok())
    {
        ros::spinOnce();

        // 判断控制指令实时性
        if( (joint_cmd.header.stamp-ros::Time::now()).toSec()>2 ){
            ROS_WARN("Joint Cmd is not realtime!");
            loop.sleep();
            continue;
        }

        if( !joint_cmd.enable ){
            for( int i=0;i<=4;i++){
                ao_register[i] = valve_close_current[i];

                uint16_t u16Data[1] = { ao_register[i]*102.4+2048 };
                modbus_write_registers(ctx, i, 1, u16Data);
            }

            loop.sleep();

            ROS_WARN("Joint Cmd is not enable!");
            continue;
        }

        // 0-20mA   0-4095寄存器
        for( int i=0;i<8;i++){
            ao_register[i] = joint_cmd.current[i]*102.4+2048;

            uint16_t u16Data[1] = { ao_register[i] };

            ROS_INFO("register: %d, data: %d", i, u16Data[0]);
            modbus_write_registers(ctx, i, 1, u16Data);
        } 

        loop.sleep();
    }

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mcs_robot_node");

    Modbus_Master master;

    return 0;
}