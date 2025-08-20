#include <iostream>
#include <yaml-cpp/yaml.h>
#include "src/observer.h"

int main(int argc, char** argv) {

    YAML::Node config = YAML::LoadFile("../config/config.yaml");

    Observer observer(config);

    while (true) {
        observer.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
