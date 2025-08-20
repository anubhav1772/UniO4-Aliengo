/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include "state_estimator_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "pd_tau_targets_lcmt.hpp"
#include "rc_command_lcmt.hpp"
#include "PositionGravityState.hpp"

#include "./HDF5_recorder.h"
//#include "./T265_reader.h"
#include "./utils.h"

// #include "camera_lcm_msgs/PositionGravityState.hpp"

#include <csignal>  // Needed for signal

using namespace std; 
using namespace UNITREE_LEGGED_SDK;
// using namespace camera_lcm_msgs;

// low cmd
constexpr uint16_t TARGET_PORT = 8007;
constexpr uint16_t LOCAL_PORT = 8082;
constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

const int LOW_CMD_LENGTH = 610;
const int LOW_STATE_LENGTH = 771;

// const float PosStopF = 2.146e+9f;
// const float VelStopF = 16000.0f;

const float MAX_TORQUE = 15.0f;

HDF5Recorder HDF5Recorder;
//RealSensePose T265_reader;


// class PositionGravityStateHandler {

//     public:
//         std::vector<float> gravity;
//         std::vector<float> states_step;
    
//         void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const PositionGravityState* msg) {
//             //gravity(msg->gravity, msg->gravity + 3);
//             for(int i=0; i<3; i++)
//             {
//                 gravity.push_back(msg->gravity[i]);
//             }
//             cout << "Received Gravity Vector: [" << gravity[0] << ", " << gravity[1] << ", " << gravity[2] << "]" << std::endl;

//             for(int i=0; i<18; i++)
//             {
//                 states_step.push_back(msg->data[i]);
//             }
//             //std::cout << "Received states_step vector:" << std::endl;
//         }
//     };

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

    void handleMessageLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const PositionGravityState* msg);

    void init();
    void handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg);
    void _simpleLCMThread();

    float record_state(float data, int scale);
    float record_action(float data);

    bool processBoolean(int8_t value);

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

    lcm::LCM _simpleLCM;
    std::thread _simple_LCM_thread;
    bool _firstCommandReceived;
    bool _firstRun;
    state_estimator_lcmt body_state_simple = {0};
    leg_control_data_lcmt joint_state_simple = {0};
    pd_tau_targets_lcmt joint_command_simple = {0};
    rc_command_lcmt rc_command = {0};

    xRockerBtnDataStruct _keyData;
    int mode = 0;

    vector<float> actions_step;
    vector<float> states_step;
    int counter = 0;

    //PositionGravityStateHandler handler;
    std::vector<float> gravity;
    std::vector<float> steps;
    PositionGravityState camera_state;
};

void Custom::handleMessageLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const PositionGravityState* msg) {

    // std::cout<<"HELLO!!"<<std::endl;
    (void) rbuf;
    (void) chan;

    gravity.clear();
    steps.clear();

    //gravity(msg->gravity, msg->gravity + 3);
    for(int i=0; i<3; i++)
    {
        gravity.push_back(msg->gravity[i]);
    }
    // cout << "Received Gravity Vector: [" << gravity[0] << ", " << gravity[1] << ", " << gravity[2] << "]" << std::endl;

    for(int i=0; i<18; i++)
    {
        steps.push_back(msg->data[i]);
    }
    // std::cout << "Received states_step vector: [";
    // for (size_t i = 0; i < steps.size(); ++i) {
    //     std::cout << steps[i];
    //     if (i < steps.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "]" << std::endl;
}

void Custom::init()
{
    
    _simpleLCM.subscribe("POSITION_GRAVITY_STATE", &Custom::handleMessageLCM, this);
    _simpleLCM.subscribe("pd_plustau_targets", &Custom::handleActionLCM, this);
    _simple_LCM_thread = std::thread(&Custom::_simpleLCMThread, this);

    _firstCommandReceived = false;
    _firstRun = true;

    // set nominal pose

    for(int i = 0; i < 12; i++)
    {
        // joint_command_simple.qd_des[i] = 0;
        // joint_command_simple.tau_ff[i] = 0;
        // joint_command_simple.kp[i] = 20.0;
        // joint_command_simple.kd[i] = 2.0;

        joint_command_simple.qd_des[i] = 0;
        joint_command_simple.tau_ff[i] = 0;
        joint_command_simple.kp[i] = 30.0;
        joint_command_simple.kd[i] = 0.8;
    }

    joint_command_simple.q_des[0] = -0.3;
    joint_command_simple.q_des[1] = 1.2;
    joint_command_simple.q_des[2] = -2.721;
    joint_command_simple.q_des[3] = 0.3;
    joint_command_simple.q_des[4] = 1.2;
    joint_command_simple.q_des[5] = -2.721;
    joint_command_simple.q_des[6] = -0.3;
    joint_command_simple.q_des[7] = 1.2;
    joint_command_simple.q_des[8] = -2.721;
    joint_command_simple.q_des[9] = 0.3;
    joint_command_simple.q_des[10] = 1.2;
    joint_command_simple.q_des[11] = -2.721;

    joint_command_simple.calibrated = 0;

    printf("SET NOMINAL POSE");
}

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

void Custom::handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg)
{
    (void) rbuf;
    (void) chan;

    joint_command_simple = *msg;
    _firstCommandReceived = true;

}

void Custom::_simpleLCMThread()
{
    while(true)
    {
        _simpleLCM.handle();
    }
}

float Custom::record_state(float data, int scale = 1)
{
    states_step.push_back(data*scale);
    return data;
}

float Custom::record_action(float data)
{   
    actions_step.push_back(data);
    return data;
}

bool Custom::processBoolean(int8_t value) {
    bool booleanValue = (value != 0);
    return booleanValue;
}

void Custom::RobotControl()
{
    states_step.clear();
    actions_step.clear();
    
    udp.GetRecv(state);

    memcpy(&_keyData, &state.wirelessRemote[0], 40);
    if (_keyData.btn.components.R1 && !rc_command.right_upper_switch)
    {
        // std::cout<<"Pressed"<<std::endl;
        HDF5Recorder.new_episode();
        // right_upper_switch_pressed = false;
    }
    rc_command.left_stick[0] = record_state(_keyData.lx)/0.6;//y speed
    rc_command.left_stick[1] = record_state(_keyData.ly)/3.5;//x speed
    // std::cout<<"cmd x:"<<_keyData.ly<<"   "<<"cmd y:"<<_keyData.lx<<std::endl;
    // std::cout<<"cmd x2:"<<record_state(_keyData.ly)<<"   "<<"cmd y2:"<<record_state(_keyData.lx)<<std::endl;
    rc_command.right_stick[1] = record_state(_keyData.ry, -1)/5.0;//yaw speed
    rc_command.right_stick[0] = record_state(_keyData.rx);//z speed
    rc_command.right_lower_right_switch = _keyData.btn.components.R2;
    rc_command.right_upper_switch = _keyData.btn.components.R1;
    rc_command.left_lower_left_switch = _keyData.btn.components.L2;
    rc_command.left_upper_switch = _keyData.btn.components.L1;


    if(_keyData.btn.components.A > 0)
    {
        mode = 0;
    } 
    else if(_keyData.btn.components.B > 0)
    {
        mode = 1;
    }
    else if(_keyData.btn.components.X > 0)
    {
        mode = 2;
    }
    else if(_keyData.btn.components.Y > 0)
    {
        mode = 3;
    }
    else if(_keyData.btn.components.up > 0)
    {
        mode = 4;
    }
    else if(_keyData.btn.components.right > 0)
    {
        mode = 5;
    }
    else if(_keyData.btn.components.down > 0)
    {
        mode = 6;    
    }
    else if(_keyData.btn.components.left > 0)
    {
        mode = 7;
    }

    rc_command.mode = mode;

    // publish state to LCM
    for(int i = 0; i < 12; i++)
    {
        // joint_state_simple.q[i] = state.motorState[i].q;
        // joint_state_simple.qd[i] = state.motorState[i].dq;
        // joint_state_simple.tau_est[i] = state.motorState[i].tauEst;

        joint_state_simple.q[i] = record_state(state.motorState[i].q);
    }

    for(int i = 0; i < 12; i++)
    {
        joint_state_simple.qd[i] = record_state(state.motorState[i].dq);
        joint_state_simple.tau_est[i] = state.motorState[i].tauEst;
    }
    
    // record_action(state.motorState[i].tauEst);
    
    for(int i = 0; i < 4; i++)
    {
        body_state_simple.quat[i] = record_state(state.imu.quaternion[i]);
    }

    for(int i = 0; i < 3; i++)
    {
        body_state_simple.rpy[i] = record_state(state.imu.rpy[i]);
        body_state_simple.aBody[i] = record_state(state.imu.accelerometer[i]);
        body_state_simple.omegaBody[i] = record_state(state.imu.gyroscope[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        body_state_simple.contact_estimate[i] = record_state(state.footForce[i]);
    }

    // std::vector<float> euler = T265_reader.appendPoseData(states_step);

    // std::vector<float> gravity = compute_gravity_vector(euler[0], euler[1], euler[2]);
    // std::cout << "Gravity in robot frame: [" << gravity[0] << ", " << gravity[1] << ", " << gravity[2] << "]" << std::endl;
    // record_state(gravity[0]);
    // record_state(gravity[1]);
    // record_state(gravity[2]);
    
    //lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");

    //PositionGravityStateHandler handler;
    // lcm.subscribe("POSITION_GRAVITY_STATE", &PositionGravityStateHandler::handleMessage, &handler);

    //_simpleLCM.subscribe("POSITION_GRAVITY_STATE", &PositionGravityStateHandler::handleMessage, &handler);

    if (steps.size()==18)
    {
        for (int i = 0; i < 18; i++) {
            states_step.push_back(steps[i]);
            camera_state.data[i] = steps[i];
        }
        cout << "steps: " << steps[2] <<" "<< steps[9];
    }
    
    if (gravity.size()==3)
    {
        record_state(gravity[0]);
        record_state(gravity[1]);
        record_state(gravity[2]);
        for (int i = 0; i < 3; i++) {
            
            camera_state.gravity[i] = gravity[i];
        }
        cout << " gravity: " << gravity[0];
    }

    _simpleLCM.publish("state_estimator_data", &body_state_simple);
    _simpleLCM.publish("leg_control_data", &joint_state_simple);
    _simpleLCM.publish("rc_command", &rc_command);
    _simpleLCM.publish("camera_python", &camera_state);

    if(_firstRun && joint_state_simple.q[0] != 0)
    {
        for(int i = 0; i < 12; i++)
        {
            joint_command_simple.q_des[i] = joint_state_simple.q[i];
        }
        _firstRun = false;
    }

    for(int i = 0; i < 12; i++)
    {
        record_action(joint_command_simple.qd_des[i]);
    }

    for(int i = 0; i < 12; i++){
       
        /*Torque Mode*/ 

        // cmd.motorCmd[i].mode = 1;
        cmd.motorCmd[i].q = record_action(joint_command_simple.q_des[i]); // 2.146E+9f
        cmd.motorCmd[i].dq = 0; // 16000.0f
        cmd.motorCmd[i].Kp = joint_command_simple.kp[i];
        cmd.motorCmd[i].Kd = joint_command_simple.kd[i];
        cmd.motorCmd[i].tau = 0;

        //cout << joint_command_simple.q_des[i] << ",";
    }
    //ccout<<endl;
    
    // if(!processBoolean(joint_command_simple.calibrated))
    // {
    //     safe.PositionLimit(cmd);
    //     // int res1 = safe.PowerProtect(cmd, state, 9);
    //     safe.PowerProtect(cmd, state, 9);
        
    //     udp.SetSend(cmd);
    // }

    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 9);
        
    udp.SetSend(cmd);
    
    if(counter >= 10)
    {
        HDF5Recorder.record_step(actions_step, states_step);
        counter = 0;
    }
    else
    {
        counter++;
    }

    // for(auto vel: states_step){
    //     std::cout<<vel<< "  ";
    // }
    // for(int i = 0; i<4; i++){
    //     std::cout<<states_step[i]<<"  ";
    // }
    // std::cout<<std::endl;
}


int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    custom.init();
    // InitEnvironment();
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
