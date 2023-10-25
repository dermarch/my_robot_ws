#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "math.h"
#include "can_drive/ICANCmd.h"
#include <tf/tf.h>
#include <bitset>
#include <string>
#include <algorithm> 
#include <thread>
#include <robot_msgs/Pump_States.h>
#include <robot_msgs/Pump_Cmd.h>
#include <robot_msgs/Cylinder_States.h>
#include <robot_msgs/Cylinder_Cmd.h>
#include <robot_msgs/Joint_States.h>
#include <robot_msgs/Joint_Cmd.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define __countof(a)  (sizeof(a)/sizeof(a[0]))
using namespace std;

struct Byte4{
    unsigned char b0;
    unsigned char b1;
    unsigned char b2;
    unsigned char b3;
};

class Like_Can
{
    private:
    DWORD dwDeviceHandle;           //can设备句柄

    const double joint_states_A1[5] = {0.1,0.1,0.1,0.1,0.1};
    const double joint_states_A2[5] = {0.1,0.1,0.1,0.1,0.1};

    const BYTE pump_motor_acc = 0x10;
    const double pump_motor_k = 2.73;           // 8192/3000
    const double trans_bar = 0.01;
    const double trans_hz = 0.01;
    const double trans_mm = 0.1;
    const double trans_N = 0.1;


    public:
    ros::NodeHandle nh;

    // pub topics
    ros::Publisher pub_pump_states;
    ros::Publisher pub_cylinder_states;
    ros::Publisher pub_joint_states;

    robot_msgs::Pump_States pump_states;
    robot_msgs::Joint_States joint_states;
    robot_msgs::Cylinder_States cylinder_states;


    // subs
    ros::Subscriber sub_joint_cmd;
    ros::Subscriber sub_pump_cmd;
    ros::Subscriber sub_cylinder_cmd;

    robot_msgs::Pump_Cmd pump_cmd;
    robot_msgs::Joint_Cmd joint_cmd;
    robot_msgs::Cylinder_Cmd cylinder_cmd;

    void joint_cmd_callback(const robot_msgs::Joint_Cmd::ConstPtr &msg);
    void pump_cmd_callback(const robot_msgs::Pump_Cmd::ConstPtr &msg);
    void cylinder_cmd_callback(const robot_msgs::Cylinder_Cmd::ConstPtr &msg);


    // controllers
    bool pump_control();                                        // 泵站控制
    bool cylinder_control();

    // funcs
    bool Init_Can();                                            // 初始化can卡
    bool Can_Recv0();                                           // can0接收并解析为相应的话题 
    void SendLoopCmd();                                         // 周期性指令，用于查询
    void SendOnceCmd();                                         // 发送一次指令，用于查询
    CAN_DataFrame can_frame_set(uint id, BYTE data[8]);         // 设置can_frame
    Byte4 int2byte4(int data);                                  // 将int数据转为4个byte，用于can发送
    int get_signed_candata(int can_data, int n);                // 解析有符号位的can数据

    // 类初始化
    Like_Can()
    {   
        sub_joint_cmd   = nh.subscribe("/joint_cmd", 10, &Like_Can::joint_cmd_callback, this);
        sub_pump_cmd    = nh.subscribe("/pump_cmd", 10, &Like_Can::pump_cmd_callback, this);
        sub_cylinder_cmd    = nh.subscribe("/cylinder_cmd", 10, &Like_Can::cylinder_cmd_callback, this);

        pub_pump_states = nh.advertise<robot_msgs::Pump_States>("/pump_states",10);
        pub_joint_states = nh.advertise<robot_msgs::Joint_States>("/joint_states",10);
        pub_cylinder_states = nh.advertise<robot_msgs::Cylinder_States>("/cylinder_states",10);

        // 给关节数据赋初值
        for( int i=0;i<5;i++){
            joint_states.name.push_back("J"+to_string(i));
            joint_states.position.push_back(0);
            joint_states.velocity.push_back(0);
            joint_states.P1.push_back(0);
            joint_states.P2.push_back(0);
            joint_states.PL.push_back(0);
        }

    }
};

