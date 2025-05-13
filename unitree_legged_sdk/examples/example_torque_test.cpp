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
    //IMU imu;
    Cartesian pose;
    //float qInit[3] = {0};
    float qInit[12] = {0};
    //float qDes[3] = {0};
    float qDes[12] = {0};
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

    //printf("Motion Time: %d\n", motiontime);


    //printf("REAR LEFT: [%f  %f %f]\t [%f  %f %f]\t [%f  %f %f]\n", state.motorState[RL_0].q, state.motorState[RL_0].dq, state.motorState[RL_0].ddq, state.motorState[RL_1].q, state.motorState[RL_1].dq, state.motorState[RL_1].ddq, state.motorState[RL_2].q, state.motorState[RL_2].dq, state.motorState[RL_2].ddq);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -1.6f;
    cmd.motorCmd[FL_0].tau = +1.0f;
    cmd.motorCmd[RR_0].tau = -1.0f;
    cmd.motorCmd[RL_0].tau = +1.0f;

    // if( motiontime >= 100){
    if (motiontime >= 0)
    {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if (motiontime >= 0 && motiontime < 10)
        {
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;

            qInit[3] = state.motorState[FL_0].q;
            qInit[4] = state.motorState[FL_1].q;
            qInit[5] = state.motorState[FL_2].q;

            qInit[6] = state.motorState[RR_0].q;
            qInit[7] = state.motorState[RR_1].q;
            qInit[8] = state.motorState[RR_2].q;
            
            qInit[9] = state.motorState[RL_0].q;
            qInit[10] = state.motorState[RL_1].q;
            qInit[11] = state.motorState[RL_2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if (motiontime >= 10 && motiontime < 400)
        {
            rate_count++;
            double rate = rate_count / 200.0; // needs count to 200
            // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
            // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            Kp[0] = 50.0;
            Kp[1] = 50.0;
            Kp[2] = 50.0;
            Kd[0] = 2.0;
            Kd[1] = 2.0;
            Kd[2] = 2.0;

            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

            qDes[3] = jointLinearInterpolation(qInit[3], sin_mid_q[0], rate);
            qDes[4] = jointLinearInterpolation(qInit[4], sin_mid_q[1], rate);
            qDes[5] = jointLinearInterpolation(qInit[5], sin_mid_q[2], rate);

            qDes[6] = jointLinearInterpolation(qInit[6], sin_mid_q[0], rate);
            qDes[7] = jointLinearInterpolation(qInit[7], sin_mid_q[1], rate);
            qDes[8] = jointLinearInterpolation(qInit[8], sin_mid_q[2], rate);

            qDes[9] = jointLinearInterpolation(qInit[9], sin_mid_q[0], rate);
            qDes[10] = jointLinearInterpolation(qInit[10], sin_mid_q[1], rate);
            qDes[11] = jointLinearInterpolation(qInit[11], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        float freq_Hz = 1;
        // float freq_Hz = 5;
        float freq_rad = freq_Hz * 2 * M_PI;
        float t = dt * sin_count;
        if (motiontime >= 400)
        {
            sin_count++;
            // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
            sin_joint1 = 0.6 * sin(t * freq_rad);
            sin_joint2 = -0.9 * sin(t * freq_rad);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];

            qDes[3] = sin_mid_q[0];
            qDes[4] = sin_mid_q[1] + sin_joint1;
            qDes[5] = sin_mid_q[2] + sin_joint2;

            qDes[6] = sin_mid_q[0];
            qDes[7] = sin_mid_q[1] + sin_joint1;
            qDes[8] = sin_mid_q[2] + sin_joint2;

            qDes[9] = sin_mid_q[0];
            qDes[10] = sin_mid_q[1] + sin_joint1;
            qDes[11] = sin_mid_q[2] + sin_joint2;
        }

        // FRONT RIGHT
        float torque = (qDes[0] - state.motorState[0].q) * Kp[0] + (0 - state.motorState[0].dq) * Kd[0] - 1.6f;
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FR_0].q = PosStopF;
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

        // FRONT LEFT
        torque = (qDes[3] - state.motorState[3].q) * Kp[0] + (0 - state.motorState[3].dq) * Kd[0] + 1.6f;
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FL_0].q = PosStopF;
        cmd.motorCmd[FL_0].dq = VelStopF;
        cmd.motorCmd[FL_0].Kp = 0;
        cmd.motorCmd[FL_0].Kd = 0;
        cmd.motorCmd[FL_0].tau = torque;

        torque = (qDes[4] - state.motorState[4].q) * Kp[1] + (0 - state.motorState[4].dq) * Kd[1];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[FL_1].q = PosStopF;
        cmd.motorCmd[FL_1].dq = VelStopF;
        cmd.motorCmd[FL_1].Kp = 0;
        cmd.motorCmd[FL_1].Kd = 0;
        cmd.motorCmd[FL_1].tau = torque;

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

        // REAR RIGHT

        torque = (qDes[6] - state.motorState[6].q) * Kp[0] + (0 - state.motorState[6].dq) * Kd[0] - 1.6f;
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;
        
        cmd.motorCmd[RR_0].q = PosStopF;
        cmd.motorCmd[RR_0].dq = VelStopF;
        cmd.motorCmd[RR_0].Kp = 0;
        cmd.motorCmd[RR_0].Kd = 0;
        cmd.motorCmd[RR_0].tau = torque;

        torque = (qDes[7] - state.motorState[7].q) * Kp[1] + (0 - state.motorState[7].dq) * Kd[1];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;

        cmd.motorCmd[RR_1].q = PosStopF;
        cmd.motorCmd[RR_1].dq = VelStopF;
        cmd.motorCmd[RR_1].Kp = 0;
        cmd.motorCmd[RR_1].Kd = 0;
        cmd.motorCmd[RR_1].tau = torque;
        
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

        // REAR LEFT
        torque = (qDes[9] - state.motorState[9].q) * Kp[0] + (0 - state.motorState[9].dq) * Kd[0] + 1.6f;
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;
        cmd.motorCmd[RL_0].q = PosStopF;
        cmd.motorCmd[RL_0].dq = VelStopF;
        cmd.motorCmd[RL_0].Kp = 0;
        cmd.motorCmd[RL_0].Kd = 0;
        cmd.motorCmd[RL_0].tau = torque;

        torque = (qDes[10] - state.motorState[10].q) * Kp[1] + (0 - state.motorState[10].dq) * Kd[1];
        if (torque >= 15)
            torque = 15;
        else if (torque <= -15)
            torque = -15;
        cmd.motorCmd[RL_1].q = PosStopF;
        cmd.motorCmd[RL_1].dq = VelStopF;
        cmd.motorCmd[RL_1].Kp = 0;
        cmd.motorCmd[RL_1].Kd = 0;
        cmd.motorCmd[RL_1].tau = torque;

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
    }

    cout << qDes[0] << "," << state.motorState[0].q << "," << qDes[1] << "," << state.motorState[1].q << "," << qDes[2] << "," << state.motorState[2].q <<endl;


    if(motiontime > 10){
        // safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 4);
        // safe.PositionProtect(cmd, state, 0.087);
    }

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
