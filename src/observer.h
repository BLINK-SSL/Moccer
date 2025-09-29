#pragma once
#include <iostream>
#include <vector>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include "networks/sender.h"
#include "networks/receiver.h"
#include "planner/planner.h"

/**
 * @class Observer
 * @brief Main controller class for RoboCup SSL Vision
 *
 * The Observer class manages:
 * - Loading configuration from a YAML file
 * - Network communication (Sender / Receiver)
 * - Planner instances for each robot
 * - Latest robot frame data (our robots and opponent robots)
 */
class Observer {
public:
    /**
     * @brief Constructor
     * @param config YAML::Node containing configuration settings
     *
     * Initializes Sender and Receiver, creates and starts Planner instances
     * based on the configuration.
     */
    Observer(const YAML::Node& config);

    /**
     * @brief Destructor
     *
     * Stops all Planner instances and the Receiver.
     */
    ~Observer();

    /**
     * @brief Update for a single frame
     *
     * Fetches the latest frame from the Receiver, updates the Planners,
     * and sends commands through the Sender.
     */
    void update();

private:
    /// YAML configuration copy
    const YAML::Node conf;

    /// Network sender for transmitting commands
    Sender sender;

    /// Network receiver for receiving data
    Receiver receiver;

    /// Collection of Planner instances
    vector<unique_ptr<Planner>> planners;

    /// Maximum number of robots
    int maxRobotCount;

    /// Mutex for thread-safe access to robot data
    mutex mtx;

    /// Latest frame of our robots
    vector<Robot> ourRobots;

    /// Latest frame of opponent robots
    vector<Robot> enemyRobots;
};
