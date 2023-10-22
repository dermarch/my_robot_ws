#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "math.h"
#include "can_drive/ICANCmd.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <bitset>
#include <string>
#include <algorithm> 
#include <thread>
#include <can_drive/canbus.h>

int main(int argc, char **argv)
{
    // ros节点初始化
    ros::init(argc,argv,"can_drive_node");

    Like_Can CAN;

    if( CAN.Init_Can() ){
		ROS_WARN("Init: can init success!");
	}else{
		ROS_WARN("Init: can init failed, please check and restart!");
		exit(0);
	}


    ros::Rate loop_rate(20);

    while(ros::ok())
    {   
	    ros::spinOnce();                // 根据接收的话题计算要发送的 Can_Frame 并发送相应的 can_frame

        CAN.SendLoopCmd();
        // can0 receive and pub topics
        if (!CAN.Can_Recv0()){
            ROS_WARN("can0 receive wrong, try to restart");
            cout<< "Init Can Result: "<< CAN.Init_Can() <<endl;
            continue;
        }

        // pump_cmd
        if( !CAN.pump_control() ){
            ROS_WARN("pump control cmd send wrong!");
        }



        // hsv_cmd



        // servo_valve_cmd



        loop_rate.sleep();
    }
}
