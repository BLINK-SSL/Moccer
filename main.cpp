#include <iostream>
#include "src/observer.h"

int main(int argc, char** argv) {
    Observer observer;
    
    while (true) {
        observer.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
