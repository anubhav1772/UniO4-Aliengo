// /*****************************************************************
//  Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// ******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    // printf("%d   %f\n", motiontime, state.forwardSpeed);
    // printf("%f %f \n", state.imu.quaternion[0], state.imu.quaternion[1]);

    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.dBodyHeight = 0.0f;

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.rpy[0]  = 0;
    cmd.rpy[1] = 0;
    cmd.rpy[2] = 0;

    if(motiontime>1000 && motiontime<1500){
        cmd.mode = 1;
        cmd.rpy[1] = 0.5f;
    }

    if(motiontime>1500 && motiontime<2000){
        cmd.mode = 1;
        cmd.rpy[2] = 0.3f;
    }

    if(motiontime>2000 && motiontime<2500){
        cmd.mode = 1;
        cmd.rpy[3] = -0.6f;
    }

    if(motiontime>2500 && motiontime<3000){
        cmd.mode = 1;
        cmd.dBodyHeight = -0.1f;
    }

    if(motiontime>3000 && motiontime<3500){
        cmd.mode = 1;
        cmd.dBodyHeight = 0.1f;
    }

    if(motiontime>3500 && motiontime<4000){
        cmd.mode = 1;
        cmd.dBodyHeight = 0.0f;
    }

    if(motiontime>4000 && motiontime<5000){
        cmd.mode = 2;
    }

    if(motiontime>5000 && motiontime<8500){
        cmd.mode = 2;
        cmd.velocity[0] = 0.1f; // -1  ~ +1
    }

    if(motiontime>8500 && motiontime<12000){
        cmd.mode = 2;
        cmd.velocity[0] = -0.2f; // -1  ~ +1
    }

    if(motiontime>12000 && motiontime<16000){
        cmd.mode = 2;
        cmd.yawSpeed = 0.3f;   // turn
    }

    if(motiontime>16000 && motiontime<20000){
        cmd.mode = 2;
        cmd.yawSpeed = -0.3f;   // turn
    }

    if(motiontime>20000 ){
        cmd.mode = 1;
    }

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
//    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
