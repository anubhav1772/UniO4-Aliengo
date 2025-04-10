#include <lcm/lcm-cpp.hpp>
//#include "camera_lcm_msgs/PositionGravityState.hpp"
#include "camera_lcm_msgs_PositionGravityState.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using namespace std;
using namespace UNITREE_LEGGED_SDK;
//typedef struct _camera_lcm_msgs_PositionGravityState camera_lcm_msgs_PositionGravityState;

class PositionGravityStateHandler {
    public:
        void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const camera_lcm_msgs_PositionGravityState::camera_lcm_msgs_PositionGravityState* msg) {
            std::vector<float> gravity(msg->gravity, msg->gravity + 3);
            std::cout << "Received Gravity Vector: [" << gravity[0] << ", " << gravity[1] << ", " << gravity[2] << "]" << std::endl;

            std::vector<float> states_step(msg->data, msg->data + 18);
            std::cout << "Received states_step vector:" << std::endl;
        }
    };
    
vector<float> states_step;

int main() {
    // This will be writen instead of this part:
    /*
    std::vector<float> euler = T265_reader.appendPoseData(states_step);

    std::vector<float> gravity = compute_gravity_vector(euler[0], euler[1], euler[2]);
    
    record_state(gravity[0]);
    record_state(gravity[1]);
    record_state(gravity[2]);
    */
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");

    PositionGravityStateHandler handler;
    lcm.subscribe("POSITION_GRAVITY_STATE", &PositionGravityStateHandler::handleMessage, &handler);

    for (int i = 0; i < 18; i++) {
        states_step.push_back(data[i]);
    }

    record_state(gravity[0]);
    record_state(gravity[1]);
    record_state(gravity[2]);

    while (lcm.handle() == 0);
    return 0;
}