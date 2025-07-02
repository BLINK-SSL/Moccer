#include "src/networks/sender.h"
#include "src/networks/receiver.h"
#include <iostream>

using namespace std::chrono;

int main(int argc, char *argv[]) {
    Sender sender;
    Receiver receiver;
    receiver.start();
    auto sendInterval = milliseconds(16);
    auto lastSendTime = steady_clock::now();
    float angle = 0.0;
    while (true) {
        Robot* robots = receiver.getBlueRobots();
        // std::cout << robots[0].orientation*180/3.14 << std::endl;
        // for (int i = 0; i < 16; ++i) {
            // std::cout << "Blue Robot ID: " << i << ", X: " << robots[i].x << ", Y: " << robots[i].y << std::endl;
        // }
        auto now = steady_clock::now();
        if (now - lastSendTime >= sendInterval) {
            sender.send(false);
            lastSendTime = now;
        }
    }
    receiver.stop();
}
