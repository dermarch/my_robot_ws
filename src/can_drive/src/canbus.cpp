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
    int len = __countof(data);

    frame.nSendType = 0;      // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    frame.bRemoteFlag = 0;    // 0-数据帧；1-远程帧
    frame.bExternFlag = 0;    // 0-标准帧；1-扩展帧
    frame.nDataLen = len;       // DLC

    frame.uID = id;
    for (int j=0; j<len; j++)
    {
        frame.arryData[j] = data[j];
    }
    return frame;      
};

// can_frame set
CAN_DataFrame Like_Can::can_frame_set2(uint id, BYTE data[2])
{
    CAN_DataFrame frame;
    int len = __countof(data);

    frame.nSendType = 0;      // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    frame.bRemoteFlag = 0;    // 0-数据帧；1-远程帧
    frame.bExternFlag = 0;    // 0-标准帧；1-扩展帧
    frame.nDataLen = len;       // DLC

    frame.uID = id;
    for (int j=0; j<len; j++)
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
    printf(" %02X\n", b.b0);
    printf(" %02X\n", b.b1);
    printf(" %02X\n", b.b2);
    printf(" %02X\n", b.b3);
    // std::cout<< b.b0 <<endl;
    // std::cout<< b.b1 <<endl;
    // std::cout<< b.b2 <<endl;
    // std::cout<< b.b3 <<endl;
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
    // CAN0 setting
    CAN_DataFrame can_send_OnceCmd0[1];

    // gcan数据采集卡启动
    uint id = 0x000;
    BYTE can0_data1[2] = {0x01, 0x01};
    can_send_OnceCmd0[1] = can_frame_set2(id, can0_data1);

    unsigned long sndCnt0 = CAN_ChannelSend(dwDeviceHandle, 0, can_send_OnceCmd0, 1);


    // CAN1 setting
    CAN_DataFrame can_send_OnceCmd1[1];

    // 电机进入can通信控制模式
    id = 0x00;
    BYTE can1_data1[8] = {0x01, 0x00, 0x000, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send_OnceCmd1[0] = can_frame_set(id, can1_data1);

    unsigned long sndCnt1 = CAN_ChannelSend(dwDeviceHandle, 1, can_send_OnceCmd1, 1);


}

// 需要固定发送的指令，如查询指令，使能指令，周期指令
void Like_Can::SendLoopCmd()
{   
    // CAN0 setting
    // CAN_DataFrame can_send_OnceCmd0[1];

    // // gcan数据采集卡启动
    // uint id = 0x00;
    // BYTE can0_data1[2] = {0x01, 0x01};
    // can_send_OnceCmd0[1] = can_frame_set(id, can0_data1);

    // unsigned long sndCnt0 = CAN_ChannelSend(dwDeviceHandle, 0, can_send_OnceCmd0, 1);


    CAN_DataFrame can_send_LoopCmd[2];

    // 查询电机状态
    // 20ms:0x14   50ms:0x32   2000ms:0x07d0
    uint id = 0x601;
    BYTE can_data[8] = {0x40, 0x69, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};     // 查询速度
    BYTE can_data1[8] = {0x40, 0x1C, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00};     // 查询电流
    can_send_LoopCmd[0] = can_frame_set(id, can_data);
    can_send_LoopCmd[1] = can_frame_set(id, can_data1);

    unsigned long sndCnt2 = CAN_ChannelSend(dwDeviceHandle, 1, can_send_LoopCmd, 2);
}

// 泵控制(伺服电机控制)
bool Like_Can::pump_control()
{
    uint id = 0x601;
    uint can_channel = 1;
    CAN_DataFrame pump_cmd_candata[3];

    // 检查消息实时性
    if( (pump_cmd.header.stamp - ros::Time::now()).toSec()>0.5 ){
        ROS_WARN("Pump Cmd is not realtime!");
        return 0;
    }

    // 设置伺服电机转速
    if( pump_cmd.mode == 0){
        ROS_WARN("Pump Cmd is not enable!");
        BYTE candata[8] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};   
        pump_cmd_candata[0] = can_frame_set(id, candata);
    }else{
        // step0. 进入速度控制模式
        BYTE candata0[8] = {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
        // step1. 使能
        BYTE candata1[8] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
        // step2. 设置目标速度
        int counts_speed = (int)pump_cmd.cmd*pump_motor_k;
        Byte4 byte = int2byte4(counts_speed);
        BYTE candata2[8] = {0x23, 0xff, 0x60, 0x00, byte.b0, byte.b1, byte.b2, byte.b3};
        // BYTE candata2[8] = {0x23, 0xff, 0x60, 0x00, 0x60, 0x54, 0x19, 0x00};

        // step3. 待发送内容  
        pump_cmd_candata[0] = can_frame_set(id, candata0);
        pump_cmd_candata[1] = can_frame_set(id, candata1);     
        pump_cmd_candata[2] = can_frame_set(id, candata2);  
    }

    unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle, can_channel, pump_cmd_candata, 3); 
    if( sndCnt!=3 ){
        return 0;
    }    

    return 1;  
};

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
                joint_states.P1[id]       = (DATA[3]*256 + DATA[2]) * trans_bar;                    // Pa
                joint_states.P2[id]       = (DATA[5]*256 + DATA[4]) * trans_bar;                    // Pa
                joint_states.velocity[id] = 0;
                joint_states.PL[id] = joint_states.P1[id]*joint_states_A1[id] - joint_states.P2[id]*joint_states_A2[id];
            }
            if( received[j].uID==0x205 ){
                pump_states.P0 = (DATA[1]*256 + DATA[0]) * trans_bar;
                pump_states.P1 = (DATA[3]*256 + DATA[2]) * trans_bar;                    // Pa
                pump_states.P2 = (DATA[5]*256 + DATA[4]) * trans_bar;                    // Pa
                pump_states.Q0 = (DATA[7]*256 + DATA[6]) * trans_bar;                    // mV
            }

            // 液压缸数据
            if( received[j].uID==0x206 ){
                cylinder_states.Q1       = (DATA[1]*256 + DATA[0]) * trans_hz;                    // hz
                cylinder_states.Q2       = (DATA[3]*256 + DATA[2]) * trans_hz;                    // hz
                cylinder_states.P3       = (DATA[5]*256 + DATA[4]) * trans_bar;                   // Pa
                cylinder_states.P4       = (DATA[7]*256 + DATA[6]) * trans_bar;                   // Pa
            }
            if( received[j].uID==0x207 ){
                cylinder_states.Y0       = (DATA[1]*256 + DATA[0]) * trans_mm;                    // hz
                cylinder_states.F0       = (DATA[3]*256 + DATA[2]) * trans_N;                    // hz
            }
        }

        // 发布话题
        pub_joint_states.publish(joint_states);
        pub_cylinder_states.publish(cylinder_states);

        return 1;
    }
    return 0;
};


// CAN1接收, Serve_Motor
bool Like_Can::Can_Recv1()
{   
    // 确认是否需要打印接收到的信息
    bool print_can1;
    nh.param<bool>("/can_main/print_can1", print_can1, false);

    // can接收设置
    int rec_len = 0, DATA[8];
    CAN_DataFrame received[3000];        //接收缓存，设为3000为佳

    if((rec_len = CAN_ChannelReceive(dwDeviceHandle, 1, received, __countof(received), 20))>0){  
        for(int j=0; j<rec_len; j++){ 
            
            // 打印接收到的消息
            if( print_can1 ){
                printf("CAN%d RX ID:0x%08X", 1, received[j].uID);
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

            // 伺服电机数据 
            // e2:current e4:speed e8e9:position
            if( received[j].uID==0x581 ){
                if( DATA[1] == 0x69 && DATA[2] == 0x60 ){
                    pump_states.speed = (DATA[4] + DATA[5]*256 + DATA[6]*256*256 + DATA[7]*256*256*256)/pump_motor_k;
                }
                if( DATA[1] == 0x1C && DATA[2] == 0x22 ){
                    pump_states.current = (DATA[4] + DATA[5]*256 )/100.0;
                    ROS_WARN("DATA:%0.2d", DATA[4] + DATA[5]*256);
                }
            }

        }

        // 发布话题
        pub_pump_states.publish(pump_states);

        return 1;
    }
    return 0;
};
