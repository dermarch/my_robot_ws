#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <robot_msgs/Joint_Cmd.h>
#include <robot_msgs/Pump_Cmd.h>
#include <robot_msgs/Cylinder_Cmd.h>
#include <robot_msgs/Cylinder_States.h>


class Model
{   
    private:
    const double a1 = 0;
    const double a2 = 53.5;
    const double a3 = 2.5e-5;
    const double a4 = 5;
    const double a5 = 1.0526e10;
    const double a6 = 63.1579;
    const double a7 = 8.211e3;
    const double Ps = 1e7;

    const double k_cur_axes = 20;               // [-1,1]=[-20,20]mA
    const double pump_speed_step = 20;
    
    public:  
    // joy settings
    const double joy_dead_zone = 0.2;


    // ros:
    ros::NodeHandle nh;

    // subs
    ros::Subscriber sub_cylinder_cmd;

    // pubs
    ros::Publisher pub_joint_cmd;
    ros::Publisher pub_cylinder_cmd;
    ros::Publisher pub_pump_cmd;
    ros::Publisher pub_cylinder_sta;

    robot_msgs::Joint_Cmd joint_cmd;
    robot_msgs::Pump_Cmd pump_cmd;
    robot_msgs::Cylinder_Cmd cylinder_cmd;
    robot_msgs::Cylinder_States cylinder_states;

    // subs
    ros::Subscriber sub_joy;
    void cmd_Callback(const robot_msgs::Cylinder_Cmd::ConstPtr msg);                           // 接收手柄指令

    Model();
    ~Model(){};
    double sign(double x);
};

double Model::sign( double x)
{
    if(x>0){
        return 1;
    }else if(x<0){
        return -1;
    }else{
        return 0;
    }
};

void Model::cmd_Callback(const robot_msgs::Cylinder_Cmd::ConstPtr msg)
{
    cylinder_cmd = *msg;
};

Model::Model()
{
    sub_cylinder_cmd = nh.subscribe("/cylinder_cmd", 100, &Model::cmd_Callback, this);

    // pub_joint_cmd = nh.advertise<robot_msgs::Joint_Cmd>( "/joint_cmd", 10 );
    // pub_pump_cmd = nh.advertise<robot_msgs::Pump_Cmd>( "/pump_cmd", 10 );
    // pub_cylinder_cmd = nh.advertise<robot_msgs::Cylinder_Cmd>( "/cylinder_cmd", 10 );

    pub_cylinder_sta = nh.advertise<robot_msgs::Cylinder_States>( "/cylinder_states", 10 );


    ros::Rate loop_rate(100);
    double iter = 0;
    double x1, x2, x3;
    double dx1, dx2, dx3;

    while(ros::ok())
    {
        ros::spinOnce();

        double uk = 0;
        uk = cylinder_cmd.current1;

        double ts = 0.0001;
        for(int i=0; i<100; i++){
            dx1 = x2;
            dx2 = -a1*x1-a2*x2+a3*x3-a4*sign(x2);
            dx3 = -a5*x2-a6*x3+a7*sqrt( abs(Ps-x3*sign(uk)) )*uk;

            x1 = x1 + dx1*ts;
            x2 = x2 + dx2*ts;
            x3 = x3 + dx3*ts;
        }


        cylinder_states.Y0  = x1;
        cylinder_states.vel = x2;
        cylinder_states.P3  = x3;

        ROS_INFO("sta: %.3f, %.3f, %.3f, con: %.3f", x1,x2,x3,uk);


        // 发布消息
        pub_cylinder_sta.publish(cylinder_states);
        loop_rate.sleep();
    }

};



int main (int argc, char** argv)
{
    ros::init(argc, argv, "cyl_model_node");

    Model model;

    return 0;
}