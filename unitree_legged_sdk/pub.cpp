#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "MessageA.hpp"
#include "MessageB.hpp"

int main() {
    lcm::LCM lcm;

    if (!lcm.good()) {
        std::cerr << "Failed!" << std::endl;
        return 1;
    }

    MessageA msgA;
    msgA.id = 1;
    for (int i = 0; i < 10; i++) {
        msgA.data[i] = i;
    }

    MessageB msgB;
    msgB.id = 2;
    for (int i = 0; i < 2; i++) {
        msgB.gravity[i] = 0;
    }
    msgB.gravity[2] = 9.81;

    while (true) {
        lcm.publish("CHANNEL_A", &msgA);
        std::cout << "Message A Published" << std::endl;

        lcm.publish("CHANNEL_B", &msgB);
        std::cout << "Message B Published" << std::endl;

        lcm.handleTimeout(2);
    }

    return 0;
}