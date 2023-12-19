#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <robot_msgs/Joint_Cmd.h>
#include <robot_msgs/Pump_Cmd.h>
#include <robot_msgs/Cylinder_Cmd.h>
#include <robot_msgs/Cylinder_States.h>


class Cmd_pub
{   
    private:
    const double k_cur_axes = 20;               // [-1,1]=[-20,20]mA
    const double pump_speed_step = 20;
    
    public:  
    // joy settings
    const double joy_dead_zone = 0.2;


    // ros:
    ros::NodeHandle nh;

    // subs
    ros::Subscriber sub_cylinder_states;

    // pubs
    ros::Publisher pub_joint_cmd;
    ros::Publisher pub_pump_cmd;
    ros::Publisher pub_cylinder_cmd;
    ros::Publisher pub_cylinder_sta;

    robot_msgs::Joint_Cmd joint_cmd;
    robot_msgs::Pump_Cmd pump_cmd;
    robot_msgs::Cylinder_Cmd cylinder_cmd;
    robot_msgs::Cylinder_States cylinder_states;

    // subs
    ros::Subscriber sub_joy;
    void Joy_Callback(const sensor_msgs::Joy::ConstPtr msg);                           // 接收手柄指令
    sensor_msgs::Joy joy_cmd;

    Cmd_pub();
    ~Cmd_pub(){};
    void states_Callback(const robot_msgs::Cylinder_States::ConstPtr msg); 
};




void Cmd_pub::states_Callback(const robot_msgs::Cylinder_States::ConstPtr msg)
{
    cylinder_states = *msg;
};

Cmd_pub::Cmd_pub()
{
    sub_cylinder_states = nh.subscribe("/cylinder_states", 100, &Cmd_pub::states_Callback, this);

    pub_joint_cmd = nh.advertise<robot_msgs::Joint_Cmd>( "/joint_cmd", 10 );
    pub_pump_cmd = nh.advertise<robot_msgs::Pump_Cmd>( "/pump_cmd", 10 );
    pub_cylinder_cmd = nh.advertise<robot_msgs::Cylinder_Cmd>( "/cylinder_cmd", 10 );

    pub_cylinder_sta = nh.advertise<robot_msgs::Cylinder_States>( "/cylinder_states", 10 );

    joint_cmd.enable = 0;
    for(int i=1;i<5;i++){
        joint_cmd.current.push_back(0);
    }

    ros::Rate loop_rate(2);
    int iter1 = 0;
    int iter2 = 0;
    float cur = -10;

    while(ros::ok())
    {
        ros::spinOnce();

        float pump_motor_speed, joint2_current;
        nh.getParam("/pump_motor_speed", pump_motor_speed);
        nh.getParam("/joint2_current", joint2_current);

        cur = -6 + 0.2*iter1;

        if( cur>=5 ){
            cur = 0;
            ROS_WARN("collect finish");

            pump_cmd.mode = 0;
            pump_cmd.cmd = 0;
            pump_cmd.header.stamp = ros::Time::now();
            pub_pump_cmd.publish( pump_cmd );

            joint_cmd.enable = 0;
            joint_cmd.current[2] = 0;
            pub_joint_cmd.publish( joint_cmd );

            loop_rate.sleep();

            continue;
        }

        iter1 = iter1 + 1;
        if( iter1%50 == 0 ){
            iter2 = iter2+1;
        }


        ROS_INFO("iter: %d, %d, %.2f", iter1, iter2, cur);

        // 判断液压缸是否到达顶端
        // if( cylinder_states.Y0>=150 ){
        //     joint2_current
        // }

        // pub pump_cmd
        pump_cmd.mode = 1;
        pump_cmd.cmd = 1000;
        pump_cmd.header.stamp = ros::Time::now();
        pub_pump_cmd.publish( pump_cmd );

        // pub joint cmd
        joint_cmd.enable = 1;
        joint_cmd.current[2] = cur;
        pub_joint_cmd.publish( joint_cmd );


        // pub cylinder_cmd
        // cylinder_cmd.mode = 1;
        // cylinder_cmd.current1 = 100;
        // cylinder_cmd.current2 = 1;
        // pub_cylinder_cmd.publish( cylinder_cmd );


        // pub cylinder_states
        // cylinder_states.Y0 = iter;
        // cylinder_states.P3 = 2000;
        // cylinder_states.P4 = 1000;
        // cylinder_states.header.stamp = ros::Time::now();
        // pub_cylinder_sta.publish(cylinder_states);


        loop_rate.sleep();
    }

};



int main (int argc, char** argv)
{
    ros::init(argc, argv, "cmd_pub_node");

    Cmd_pub pub;

    return 0;
}