#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <robot_msgs/Joint_Cmd.h>
#include <robot_msgs/Pump_Cmd.h>
#include <robot_msgs/Cylinder_Cmd.h>


class Joy_Move
{   
    private:
    const double k_cur_axes = 20;               // [-1,1]=[-20,20]mA
    
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

    Joy_Move();
    ~Joy_Move(){};
    sensor_msgs::Joy dead_zone_remove( sensor_msgs::Joy joy );
    float limit(float u, float min, float max);
};



sensor_msgs::Joy Joy_Move::dead_zone_remove(  sensor_msgs::Joy joy )
{
    for( int i=0; i<joy.axes.size(); i++){
        if( abs( joy.axes[i] ) < 0.2 && i!=2 && i!=5 ){
            joy.axes[i] = 0;
        }
    }
    return joy;
};

float Joy_Move::limit( float u, float min, float max )
{
    if(u<min){ u=min; }
    if(u>max){ u=max; }
    return u;
}


void Joy_Move::Joy_Callback(const sensor_msgs::Joy::ConstPtr msg)
{
    joy_cmd = *msg;
};

Joy_Move::Joy_Move()
{
    sub_joy = nh.subscribe("/joy", 100, &Joy_Move::Joy_Callback, this);

    pub_joint_cmd = nh.advertise<robot_msgs::Joint_Cmd>( "/joint_cmd", 10 );
    pub_pump_cmd = nh.advertise<robot_msgs::Pump_Cmd>( "/pump_cmd", 10 );
    pub_cylinder_cmd = nh.advertise<robot_msgs::Cylinder_Cmd>( "/cylinder_cmd", 10 );


    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        ros::spinOnce();

        // 检查手柄消息的实时性
        if( (ros::Time::now() - joy_cmd.header.stamp).toSec()>2 ){
            ROS_WARN("joy cmd is not update");

            loop_rate.sleep();
            continue;
        }


        // 去除手柄死区，防误触
        joy_cmd = dead_zone_remove(  joy_cmd );

        // 泵控制命令-电机转速
        if( joy_cmd.buttons[7]==1 ){
            ROS_INFO("Pump motor start!");
            pump_cmd.mode = 1;
            pump_cmd.cmd = 500;
        }
        if( joy_cmd.buttons[6]==1 ){
            ROS_INFO("Pump motor stop!");
            pump_cmd.mode = 0;
            pump_cmd.cmd = 0;
        }
        if( joy_cmd.buttons[2]==1 ){
            ROS_INFO("Pump motor speed go fast!");
            pump_cmd.cmd = limit( pump_cmd.cmd + 50, 0, 1500);
        }
        if( joy_cmd.buttons[3]==1 ){
            ROS_INFO("Pump motor speed go slow!");
            pump_cmd.cmd = limit( pump_cmd.cmd - 50, 0, 1500);
        }

        // 关节控制命令
        if( joy_cmd.axes[2]<-0.5 ){
            ROS_INFO("Robot Joint Enable!");
            joint_cmd.enable = 1;

        }else{
            joint_cmd.enable = 0;
        }
        // 按顺序写入控制电流值
        joint_cmd.current.clear();
        joint_cmd.current.push_back( joy_cmd.axes[6]*k_cur_axes );
        joint_cmd.current.push_back( joy_cmd.axes[0]*k_cur_axes );
        joint_cmd.current.push_back( joy_cmd.axes[1]*k_cur_axes );
        joint_cmd.current.push_back( joy_cmd.axes[3]*k_cur_axes );
        joint_cmd.current.push_back( joy_cmd.axes[4]*k_cur_axes );

        // 液压缸(单缸)控制命令

        // 夹爪控制


        // 发布消息
        pub_joint_cmd.publish( joint_cmd );
        pub_pump_cmd.publish( pump_cmd );
        pub_cylinder_cmd.publish( cylinder_cmd );



        loop_rate.sleep();
        // ros::spinOnce();
    }

};



int main (int argc, char** argv)
{
    ros::init(argc, argv, "joy_move_node");

    Joy_Move joy;

    return 0;
}