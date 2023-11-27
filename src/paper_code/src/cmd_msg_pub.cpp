#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <robot_msgs/Joint_Cmd.h>
#include <robot_msgs/Pump_Cmd.h>
#include <robot_msgs/Cylinder_Cmd.h>


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

    // pubs
    ros::Publisher pub_joint_cmd;
    ros::Publisher pub_pump_cmd;
    ros::Publisher pub_cylinder_cmd;

    robot_msgs::Joint_Cmd joint_cmd;
    robot_msgs::Pump_Cmd pump_cmd;
    robot_msgs::Cylinder_Cmd cylinder_cmd;

    // subs
    ros::Subscriber sub_joy;
    void Joy_Callback(const sensor_msgs::Joy::ConstPtr msg);                           // 接收手柄指令
    sensor_msgs::Joy joy_cmd;

    Cmd_pub();
    ~Cmd_pub(){};
    sensor_msgs::Joy dead_zone_remove( sensor_msgs::Joy joy );
    float limit(float u, float min, float max);
};



// sensor_msgs::Joy Cmd_Pub::dead_zone_remove(  sensor_msgs::Joy joy )
// {
//     for( int i=0; i<joy.axes.size(); i++){
//         if( abs( joy.axes[i] ) < 0.2 && i!=2 && i!=5 ){
//             joy.axes[i] = 0;
//         }
//     }
//     return joy;
// };

// float Cmd_Pub::limit( float u, float min, float max )
// {
//     if(u<min){ u=min; }
//     if(u>max){ u=max; }
//     return u;
// };


// void Cmd_Pub::Joy_Callback(const sensor_msgs::Joy::ConstPtr msg)
// {
//     joy_cmd = *msg;
// };

Cmd_pub::Cmd_pub()
{
    // sub_joy = nh.subscribe("/joy", 100, &Cmd_pub::Joy_Callback, this);

    pub_joint_cmd = nh.advertise<robot_msgs::Joint_Cmd>( "/joint_cmd", 10 );
    pub_pump_cmd = nh.advertise<robot_msgs::Pump_Cmd>( "/pump_cmd", 10 );
    pub_cylinder_cmd = nh.advertise<robot_msgs::Cylinder_Cmd>( "/cylinder_cmd", 10 );


    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        ros::spinOnce();

        pump_cmd.mode = 1;
        pump_cmd.cmd = 1000;
        pump_cmd.header.stamp = ros::Time::now();

        cylinder_cmd.mode = 1;
        cylinder_cmd.current1 = 100;
        cylinder_cmd.current2 = 1;


        // 发布消息
        // pub_joint_cmd.publish( joint_cmd );
        pub_pump_cmd.publish( pump_cmd );
        // pub_cylinder_cmd.publish( cylinder_cmd );



        loop_rate.sleep();
    }

};



int main (int argc, char** argv)
{
    ros::init(argc, argv, "cmd_pub_node");

    Cmd_pub pub;

    return 0;
}