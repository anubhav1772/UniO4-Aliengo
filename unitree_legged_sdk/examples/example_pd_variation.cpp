/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

// low cmd
constexpr uint16_t TARGET_PORT = 8007;
constexpr uint16_t LOCAL_PORT = 8082;
constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

const int LOW_CMD_LENGTH = 610;
const int LOW_STATE_LENGTH = 771;


class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::Aliengo),
                            udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH)
    {
        udp.InitCmdData(cmd);
        cmd.levelFlag = LOWLEVEL;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    // printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);

    // gravity compensation
    // cmd.motorCmd[FR_0].tau = -1.6f;
    // cmd.motorCmd[FL_0].tau = +1.0f;
    // cmd.motorCmd[RR_0].tau = -1.0f;
    // cmd.motorCmd[RL_0].tau = +1.0f;

    cmd.motorCmd[FR_2].q = -1.5f;
    cmd.motorCmd[FL_2].q = -1.5f;
    cmd.motorCmd[RR_2].q = -1.0f;
    cmd.motorCmd[RL_2].q = -1.0f;

    // if( motiontime >= 100){
    if (motiontime >= -1)
    {
        Kp[0] = 50.0;
        Kp[1] = 50.0;
        Kp[2] = 50.0;
        Kd[0] = 2.0;
        Kd[1] = 2.0;
        Kd[2] = 2.0;

        qDes[0] = 0.1;
        qDes[1] = 1.0;
        qDes[2] = -1.5;

        // qDes[3] = 0.1;
        // qDes[4] = 1.0;
        qDes[5] = -1.5;

        // qDes[6] = 0.1;
        // qDes[7] = 1.0;
        qDes[8] = -1.5;

        // qDes[9] = 0.1;
        // qDes[10] = 1.0;
        qDes[11] = -1.5;

        float torque = (qDes[0] - state.motorState[0].q) * Kp[0] + (0 - state.motorState[0].dq) * Kd[0] - 1.6f;
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FR_0].q =PosStopF;
        cmd.motorCmd[FR_0].dq = VelStopF;
        cmd.motorCmd[FR_0].Kp = 0;
        cmd.motorCmd[FR_0].Kd = 0;
        cmd.motorCmd[FR_0].tau = torque;

        torque = (qDes[1] - state.motorState[1].q) * Kp[1] + (0 - state.motorState[1].dq) * Kd[1];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FR_1].q = PosStopF;
        cmd.motorCmd[FR_1].dq = VelStopF;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 0;
        cmd.motorCmd[FR_1].tau = torque;

        torque = (qDes[2] - state.motorState[2].q) * Kp[2] + (0 - state.motorState[2].dq) * Kd[2];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FR_2].q = PosStopF;
        cmd.motorCmd[FR_2].dq = VelStopF;
        cmd.motorCmd[FR_2].Kp = 0;
        cmd.motorCmd[FR_2].Kd = 0;
        cmd.motorCmd[FR_2].tau = torque;
        // cmd.motorCmd[FR_2].tau = 2 * sin(t*freq_rad);

        torque = (qDes[5] - state.motorState[5].q) * Kp[2] + (0 - state.motorState[5].dq) * Kd[2];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FL_2].q = PosStopF;
        cmd.motorCmd[FL_2].dq = VelStopF;
        cmd.motorCmd[FL_2].Kp = 0;
        cmd.motorCmd[FL_2].Kd = 0;
        cmd.motorCmd[FL_2].tau = torque;
        // cmd.motorCmd[FR_5].tau = 2 * sin(t*freq_rad);

        torque = (qDes[8] - state.motorState[8].q) * Kp[2] + (0 - state.motorState[8].dq) * Kd[2];

        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[RR_2].q = PosStopF;
        cmd.motorCmd[RR_2].dq = VelStopF;
        cmd.motorCmd[RR_2].Kp = 0;
        cmd.motorCmd[RR_2].Kd = 0;
        cmd.motorCmd[RR_2].tau = torque;
        // cmd.motorCmd[FR_2].tau = 2 * sin(t*freq_rad);

        torque = (qDes[11] - state.motorState[11].q) * Kp[2] + (0 - state.motorState[11].dq) * Kd[2];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[RL_2].q = PosStopF;
        cmd.motorCmd[RL_2].dq = VelStopF;
        cmd.motorCmd[RL_2].Kp = 0;
        cmd.motorCmd[RL_2].Kd = 0;
        cmd.motorCmd[RL_2].tau = torque;
        // cmd.motorCmd[FR_8].tau = 2 * sin(t*freq_rad);

        

  if(motiontime < 100) {
    cout << qDes[0] << "," << state.motorState[0].q << "," << qDes[1] << "," << state.motorState[1].q << "," << qDes[2] << "," << state.motorState[2].q << "," <<endl;
  }
    }

    // safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 4);
    // safe.PositionProtect(cmd, state, 0.087);

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();


    Custom custom(LOWLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}
