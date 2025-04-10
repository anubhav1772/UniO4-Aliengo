#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "camera_lcm_msgs/PositionGravityState.hpp"
// #include "camera_lcm_msgs_PositionGravityState.h"
#include "./T265_reader.h"
#include "./utils.h"

using namespace std;

int main() {
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");

    RealSensePose T265_reader;
    vector<float> states_step;

    while (true) {
        states_step.clear();
        std::vector<float> euler = T265_reader.appendPoseData(states_step);

        std::vector<float> gravity = compute_gravity_vector(euler[0], euler[1], euler[2]);

        // camera_lcm_msgs::_camera_lcm_msgs_PositionGravityState msg;

        camera_lcm_msgs::PositionGravityState msg;
        
        for (int i = 0; i < 3; i++) {
            msg.gravity[i] = gravity[i];
        }

        for (int i = 0; i < 18; i++) {
            msg.data[i] = states_step[i];
        }

        lcm.publish("POSITION_GRAVITY_STATE", &msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}