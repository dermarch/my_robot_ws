#include <termios.h>  
#include <signal.h>  
#include <cmath>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
#include <robot_msgs/Joint_Cmd.h>  
#include <robot_msgs/Pump_Cmd.h>  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>
  
#define KEYCODE_Q 0x71      
#define KEYCODE_A 0x61  
#define KEYCODE_W 0x77  
#define KEYCODE_S 0x73 
#define KEYCODE_E 0x65 
#define KEYCODE_D 0x64
#define KEYCODE_R 0x72
#define KEYCODE_F 0x66
#define KEYCODE_T 0x74
#define KEYCODE_G 0x67
#define KEYCODE_Y 0x79
#define KEYCODE_H 0x68

#define KEYCODE_Z 0x7A  
#define KEYCODE_X 0x78 
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D

  
class SmartCarKeyboardTeleopNode  
{  
    private:     
        ros::NodeHandle n_;

        robot_msgs::Joint_Cmd joint_cmd; 
        robot_msgs::Pump_Cmd pump_cmd;
        ros::Publisher pub_joint_cmd;  
        ros::Publisher pub_pump_cmd;  

        const double joint_cur_step = 0.1;
        const double pump_speed_step = 20;
  
    public:  
        SmartCarKeyboardTeleopNode()  
        {  
            pub_joint_cmd = n_.advertise<robot_msgs::Joint_Cmd>("/joint_cmd", 10);
            pub_pump_cmd = n_.advertise<robot_msgs::Pump_Cmd>("/pump_cmd", 10);
              
            ros::NodeHandle n_private("~");  
        }  
          
        ~SmartCarKeyboardTeleopNode() { }  
        void keyboardLoop();  
          
        void stopRobot()  
        {  

        }  
};

SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0;  
struct termios cooked, raw;  
bool done;  

void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;   
    bool dirty = false;   
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  

    raw.c_lflag &=~ (ICANON | ECHO);  
    /**
	 * c_lflag : 本地模式标志，控制终端编辑功能
	 * ICANON: 使用标准输入模式
	 * ECHO: 显示输入字符
	 */
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    /** 
	 * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
	 * VEOL: 附加的End-of-file字符
	 * VEOF: End-of-file字符
	 * */
    tcsetattr(kfd, TCSANOW, &raw);  

    // 初始化消息
    pump_cmd.mode = 0;
    pump_cmd.cmd = 0;
    joint_cmd.enable = 0;
    for(int i=1;i<5;i++){
        joint_cmd.current.push_back(0);
    }

      
    puts("Reading from keyboard");
      
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  
      
    for(;;)  
    {  
        boost::this_thread::interruption_point();  
          
        //get the next event from the keyboard  
        int num;  
          
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():"); 
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            // 每按下一次动一下 
            if (dirty == true)  
            {  
                stopRobot(); 
                pub_joint_cmd.publish(joint_cmd);  
                pub_pump_cmd.publish(pump_cmd);
                dirty = false;  
            }  
              
            continue;  
        }  
          
        switch(c)  
        {  
            // 机械臂使能
            case KEYCODE_Y:  
                joint_cmd.enable = true;
                dirty = true;  
                break;  
            case KEYCODE_H:  
                joint_cmd.enable = false;
                dirty = true;  
                break;  
            // 机械臂加电流
            case KEYCODE_Q:  
                joint_cmd.current[0] = joint_cmd.current[0]+joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_W:  
                joint_cmd.current[1] = joint_cmd.current[1]+joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_E:  
                joint_cmd.current[2] = joint_cmd.current[2]+joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_R:  
                joint_cmd.current[3] = joint_cmd.current[3]+joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_T:  
                joint_cmd.current[4] = joint_cmd.current[4]+joint_cur_step;
                dirty = true;  
                break;  
            // 机械臂减电流
            case KEYCODE_A:  
                joint_cmd.current[0] = joint_cmd.current[0]-joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_S:  
                joint_cmd.current[1] = joint_cmd.current[1]-joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_D:  
                joint_cmd.current[2] = joint_cmd.current[2]-joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_F:  
                joint_cmd.current[3] = joint_cmd.current[3]-joint_cur_step;
                dirty = true;  
                break;  
            case KEYCODE_G:  
                joint_cmd.current[4] = joint_cmd.current[4]-joint_cur_step;
                dirty = true;  
                break; 
            // 泵电机使能
            case KEYCODE_M:  
                pump_cmd.mode = 1;
                dirty = true;  
                break;  
            case KEYCODE_N:  
                pump_cmd.mode = 0;
                dirty = true;  
                break;  
            case KEYCODE_Z:  
                pump_cmd.cmd = pump_cmd.cmd + pump_speed_step;
                dirty = true;  
                break;  
            case KEYCODE_X:  
                pump_cmd.cmd = pump_cmd.cmd - pump_speed_step;
                dirty = true;  
                break;  

            default:  
                dirty = false;  
        }  

        joint_cmd.header.stamp = ros::Time::now();
        pump_cmd.header.stamp = ros::Time::now();

        std::cout<<"pump_cmd: "<<pump_cmd.cmd<<std::endl;
        std::cout<<"joint_cmd: "<<joint_cmd.current[2]<<std::endl;

        pub_joint_cmd.publish(joint_cmd);  
        pub_pump_cmd.publish(pump_cmd);

    }  
};
  
  
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"key_pwm_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    SmartCarKeyboardTeleopNode tbk;  
    // 创建一个新的线程 
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));  
      
    ros::spin();  
      
    t.interrupt();  
    t.join();  
    tbk.stopRobot();  
    //   设置终端参数 
    tcsetattr(kfd, TCSANOW, &cooked);  
      
    return(0);  
}  
  

