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
    while (true) {
        for (int i = 0; i < 16; ++i) {
            Robot* robots = receiver.getBlueRobots();
            // std::cout << "Blue Robot ID: " << i << ", X: " << robots[i].x << ", Y: " << robots[i].y << std::endl;
        }
        auto now = steady_clock::now();
        if (now - lastSendTime >= sendInterval) {
            sender.send(false);
            lastSendTime = now;
        }
    }
    receiver.stop();
}
