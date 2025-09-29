/**
 * @file main.cpp
 * @brief Entry point of the application.
 *
 * This program loads configuration from a YAML file, initializes
 * an Observer instance, and repeatedly calls its update function
 * in a loop.
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include "src/observer.h"
#include <thread>
#include <chrono>

/**
 * @brief Main function of the application.
 *
 * Steps:
 * - Load configuration from a YAML file
 * - Initialize the Observer instance
 * - Enter an infinite loop, calling observer.update() periodically
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return Exit code (0 for successful execution)
 */
int main(int argc, char** argv) {

    /// Load configuration from YAML file
    YAML::Node config = YAML::LoadFile("../config/config.yaml");

    /// Create an Observer instance using the loaded configuration
    Observer observer(config);

    /// Main loop: update the observer continuously
    while (true) {
        observer.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
