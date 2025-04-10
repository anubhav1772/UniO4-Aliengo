#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "MessageA.hpp"
#include "MessageB.hpp"

class Subscriber {
public:
    void messageAHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const MessageA* msg) {
        std::cout << "Message A Received: ID = " << msg->id << std::endl << "Data:" << std::endl;
        for (int i = 0; i < 10; i++) {
            std::cout << msg->data[i] << std::endl;
        }
    }

    void messageBHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const MessageB* msg) {
        std::cout << "Message B Received: ID = " << msg->id << std::endl << "Gravity:" << std::endl;
        for (int i = 0; i < 3; i++) {
            std::cout << msg->gravity[i] << std::endl;
        }
    }
};

int main() {
    lcm::LCM lcm;

    if (!lcm.good()) {
        std::cerr << "Failed!" << std::endl;
        return 1;
    }

    Subscriber subscriber;

    lcm.subscribe("CHANNEL_A", &Subscriber::messageAHandler, &subscriber);
    lcm.subscribe("CHANNEL_B", &Subscriber::messageBHandler, &subscriber);

    while (true) {
        lcm.handle();
    }

    return 0;
}