#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "camera_lcm_msgs/PositionGravityState.hpp"
#include "./T265_reader.h"
#include "./utils.h"

using namespace std;

/**int print_data(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const camera_lcm_msgs::PositionGravityState* msg){
	vector<float> gr = msg->gravity;
	cout<<gr[1];

	return -1;
}**/

int main() {
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");

    RealSensePose T265_reader;
    vector<float> states_step;
    
    //lcm.subscribe("POSITION_GRAVITY_STATE", &print_data);

    while (true) {
        states_step.clear();
        std::vector<float> euler = T265_reader.appendPoseData(states_step);

        std::vector<float> gravity = compute_gravity_vector(euler[0], euler[1], euler[2]);

        camera_lcm_msgs::PositionGravityState msg;
        
        for (int i = 0; i < 3; i++) {
            msg.gravity[i] = gravity[i];
        }

        for (int i = 0; i < 18; i++) {
            msg.data[i] = states_step[i];
        }

        lcm.publish("POSITION_GRAVITY_STATE", &msg);
	if(lcm.good()) {
	//	std::cout<<"lcm working perfectly!!";
	}

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
