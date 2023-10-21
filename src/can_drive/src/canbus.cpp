#include <can_drive/canbus.h>

// callbacks()
void Like_Can::joint_cmd_callback(const robot_msgs::Joint_Cmd::ConstPtr &msg)
{
    joint_cmd = *msg;
};

void Like_Can::pump_cmd_callback(const robot_msgs::Pump_Cmd::ConstPtr &msg)
{
    pump_cmd = *msg;
};

void Like_Can::cylinder_cmd_callback(const robot_msgs::Cylinder_Cmd::ConstPtr &msg)
{
    cylinder_cmd = *msg;
};

// can_frame set
CAN_DataFrame Like_Can::can_frame_set(uint id, BYTE data[8])
{
    CAN_DataFrame frame;

    frame.nSendType = 0;      // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    frame.bRemoteFlag = 0;    // 0-数据帧；1-远程帧
    frame.bExternFlag = 0;    // 0-标准帧；1-扩展帧
    frame.nDataLen = 8;       // DLC

    frame.uID = id;
    for (int j=0; j<8; j++)
    {
        frame.arryData[j] = data[j];
    }
    return frame;      
};

// int 转为 4个 byte，用于can发送
Byte4 Like_Can::int2byte4(int data)
{
    Byte4 b;
    b.b0 = data;
    b.b1 = (data >> 8) & 0xff;
    b.b2 = (data >> 16) & 0xff;
    b.b3 = (data >> 24) & 0xff;
    return b;
};

// 解析首位为符号位的can_data
int Like_Can::get_signed_candata(int can_data, int n)
{
    if( can_data > pow(2,n-1) )  
        can_data = can_data - pow(2,n);
    return can_data; 
};

// can卡初始化
bool Like_Can::Init_Can()         
{
    dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B,0, 0);
    if ( !CAN_DeviceOpen(ACUSB_132B,0, 0)) {
        printf("open deivce error\n");
        return 0;
    }else{
        printf(">>open deivce success\n");
    }
    //初始化参数，严格参数二次开发函数库说明书。
    CAN_InitConfig config;		
    config.dwAccCode = 0;
    config.dwAccMask = 0xffffffff;
    config.nFilter  = 0;       // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
    config.bMode    = 0;  //正常模式
    config.nBtrType = 1;
    config.dwBtr[0] = 0x00; 
    config.dwBtr[1] = 0x1c;     //500khz
    config.dwBtr[2] = 0;
    config.dwBtr[3] = 0;

    // 启动通道0     500k
    if ( CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK ) 
    {
        printf("Start CAN 0 error\n");
        return 0;
    }
    else
    {
        printf("Start CAN 0 success\n");
    };
    
    // 启动通道1     500k
    config.dwBtr[0] = 0x00; 
    config.dwBtr[1] = 0x1c;
    if ( CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK ) 
    {
        printf("Start CAN 1 error\n");
        return 0;
    }
    else
    {
        printf("Start CAN 1 success\n");
    };

    return 1;
};

// 需要固定发送一次的指令，如查询指令，使能指令，主函数中调用一次即可
void Like_Can::SendOnceCmd()
{
}

// 需要固定发送的指令，如查询指令，使能指令，周期指令
void Like_Can::SendLoopCmd()
{   
    CAN_DataFrame can_send_LoopCmd1[2];

    // 查询抓手电机编码器位置 (607)
    uint id = 0x607;
    BYTE can_data[8] = {0x40, 0x64, 0x60, 0x00, 00, 00, 00, 00};                // 查询编码器位置
    BYTE can_data1[8] = {0x40, 0x78, 0x60, 0x00, 00, 00, 00, 00};               // 查询电流
    can_send_LoopCmd1[0] = can_frame_set(id, can_data);
    can_send_LoopCmd1[1] = can_frame_set(id, can_data1);

    unsigned long sndCnt2 = CAN_ChannelSend(dwDeviceHandle, 1, can_send_LoopCmd1, 2);

}

// 机械臂关节控制(5个伺服阀)
bool Like_Can::joint_control()
{
    // 检查消息实时性
    if( (joint_cmd.header.stamp - ros::Time::now()).toSec()>0.5 ){
        ROS_WARN("Joint Cmd is not realtime!");
        return 0;
    }
    return 1;  
}

// 泵控制(伺服电机控制)
bool Like_Can::pump_control()
{
    uint id = 0x07;
    uint can_channel = 0;
    CAN_DataFrame pump_cmd_candata[2];
    // 检查消息实时性
    if( (pump_cmd.header.stamp - ros::Time::now()).toSec()>0.5 ){
        ROS_WARN("Pump Cmd is not realtime!");
        return 0;
    }

    // 转速指令解析
    Byte4 byte = int2byte4( int(pump_cmd.cmd*pump_motor_k) );

    // 设置伺服电机转速
    if( pump_cmd.mode == 0){
        ROS_WARN("Pump Cmd is not enable!");
        BYTE candata[8] = {0x00,0x1a,0x00,0x00,0x01,0x00,0x00,0x00};   
        pump_cmd_candata[0] = can_frame_set(id, candata);
    }else{
        BYTE candata0[8] = {0x00,0x1a,0x00,0x00,0x01,0x00,0x00,0x01}; 
        BYTE candata1[8] = {0x00,0x1a,0x0a,pump_motor_acc,pump_motor_acc,byte.b0,byte.b1,0x00};   
        pump_cmd_candata[0] = can_frame_set(id, candata0);
        pump_cmd_candata[1] = can_frame_set(id, candata1);     
    }

    unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle, can_channel, pump_cmd_candata, 2); 
    if( sndCnt!=2 ){
        return 0;
    }    

    return 1;  
}

// 液压缸控制(单缸)
bool Like_Can::cylinder_control()
{
    // 检查消息实时性
    if( (cylinder_cmd.header.stamp - ros::Time::now()).toSec()>0.5 ){
        ROS_WARN("Cylinder Cmd is not realtime!");
        return 0;
    }
    return 1;  
}


// CAN0接收, PLC Serve_Motor
bool Like_Can::Can_Recv0()
{   
    // 确认是否需要打印接收到的信息
    bool print_can0;
    nh.param<bool>("/can_main/print_can0", print_can0, false);

    // can接收设置
    int rec_len = 0, DATA[8];
    CAN_DataFrame received[3000];        //接收缓存，设为3000为佳

    if((rec_len = CAN_ChannelReceive(dwDeviceHandle, 0, received, __countof(received), 20))>0){  
        for(int j=0; j<rec_len; j++){ 
            
            // 打印接收到的消息
            if( print_can0 ){
                printf("CAN%d RX ID:0x%08X", 0, received[j].uID);
                printf("DATA:0x");
                for(int i = 0; i < received[j].nDataLen; i++)
                {
                    printf(" %02X", received[j].arryData[i]);
                } 
                printf("\n");
            }

            // 赋值给 DATA 用于后续的处理
            for(int i = 0; i < received[j].nDataLen; i++){
                DATA[i]=received[j].arryData[i];
            } 

            // 批处理关节数据
            if( received[j].uID==0x200 || received[j].uID==0x201 || received[j].uID==0x202 || 
                received[j].uID==0x203 || received[j].uID==0x204){
                int id = received[j].uID - 0x200;
                joint_states.position[id] = (DATA[1]*256 + DATA[0])*0.1;                // mm
                joint_states.P1[id]       = (DATA[3]*256 + DATA[2]);                    // Pa
                joint_states.P2[id]       = (DATA[5]*256 + DATA[4]);                    // Pa
                joint_states.velocity[id] = 0;
                joint_states.PL[id] = joint_states.P1[id]*joint_states_A1[id] - joint_states.P2[id]*joint_states_A2[id];
            }

            // 伺服电机数据 
            // e2:current e4:speed e8e9:position
            if( received[j].uID==0x07 ){
                if( DATA[1] == 0x88 ){
                    if( DATA[2] == 0xe2 ){
                        pump_states.current = (DATA[3]*256+DATA[4])*0.01;
                    }
                    if( DATA[5] == 0xe4 ){
                        pump_states.current = (DATA[6]*256+DATA[7])/pump_motor_k;
                    }
                    if( DATA[2] == 0xe8 &&  DATA[5] == 0xe9){
                        pump_states.count = get_signed_candata( ( DATA[7] + (DATA[6]<<8) + (DATA[4]<<16) + (DATA[3]<<24) ),32 );
                    }
                }
            }

            // 液压缸数据
            if( received[j].uID==0x205 ){
                cylinder_states.position = (DATA[1]*256 + DATA[0])*0.1;
                cylinder_states.P1       = (DATA[3]*256 + DATA[2]);                    // Pa
                cylinder_states.P2       = (DATA[5]*256 + DATA[4]);                    // Pa
            }
            if( received[j].uID==0x206 ){
                cylinder_states.Q1       = (DATA[1]*256 + DATA[0]);                    // mL/min
                cylinder_states.Q2       = (DATA[3]*256 + DATA[2]);                    // mL/min
            }
        }

        // 发布话题
        pub_joint_states.publish(joint_states);
        pub_cylinder_states.publish(cylinder_states);
        pub_pump_states.publish(pump_states);

        return 1;
    }
    return 0;
};
