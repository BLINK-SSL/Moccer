#include "src/networks/sender.h"
#include "src/networks/receiver.h"
#include <iostream>

int main(int argc, char *argv[]) {
    // Sender sender;
    Receiver receiver;
    receiver.start();
    while (true) {
        for (int i = 0; i < 16; ++i) {
            Robot* robots = receiver.getBlueRobots();
            std::cout << "Blue Robot ID: " << i << ", X: " << robots[i].x << ", Y: " << robots[i].y << std::endl;
        }
    }
    receiver.stop();
}
