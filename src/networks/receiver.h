#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "../models/robot.h"
#include "mocSim_Commands.pb.h"
#include "ssl_vision_wrapper.pb.h"
#include "ssl_vision_detection.pb.h"

using namespace std;

/**
 * @brief FPS counter utility class
 * 
 * Calculates Frames Per Second (FPS) based on the number of 
 * received frames. Runs in a separate thread and prints FPS 
 * to stdout every second.
 */
class FpsCounter {
public:
    /**
     * @brief Constructor
     * Initializes secPerFrame to 1.0 to avoid undefined values.
     */
    FpsCounter();

    /**
     * @brief Start FPS measurement in a background thread.
     */
    void start();

    /**
     * @brief Stop FPS measurement and join the thread.
     */
    void stop();

    /**
     * @brief FPS calculation loop.
     * 
     * Runs once per second, updates secPerFrame, 
     * and prints FPS to stdout.
     */
    void calculateFps();

    /**
     * @brief Get the time per frame in seconds.
     * @return Seconds per frame (double).
     */
    double getSecPerFrame();

    /// Number of frames received within the last second.
    int frameCounter;

private:
    /// Seconds per frame (calculated once per second).
    double secPerFrame;

    /// Mutex to protect secPerFrame access.
    mutex secPerFrameMutex;

    /// Start time reference point.
    chrono::high_resolution_clock::time_point startTime;

    /// Worker thread for FPS calculation.
    thread fpsThread;

    /// Running flag for background thread.
    atomic<bool> running_;
};


/**
 * @brief Receiver class for SSL-Vision packets.
 * 
 * Opens a UDP multicast socket, receives SSL-Vision data, 
 * updates robot states, and provides access to our and enemy robots.
 */
class Receiver {
public:
    /**
     * @brief Construct a new Receiver object.
     * 
     * Sets up the UDP socket, joins the multicast group, 
     * and prepares robot containers.
     * 
     * @param config YAML configuration node.
     */
    Receiver(const YAML::Node& config);

    /**
     * @brief Destructor. Automatically calls stop().
     */
    ~Receiver();

    /**
     * @brief Start the receiving thread.
     */
    void start();

    /**
     * @brief Stop the receiving thread and the FPS counter.
     */
    void stop();

    /**
     * @brief Get the latest packet (currently unused).
     */
    SSL_WrapperPacket getLatestPacket();

    /**
     * @brief Get our team robots.
     * @return Vector of Robot (copy).
     */
    vector<Robot> getOurRobots();

    /**
     * @brief Get enemy team robots.
     * @return Vector of Robot (copy).
     */
    vector<Robot> getEnemyRobots();

    /// Capture timestamp of the latest Vision packet.
    double tCapture;

    /// Sent timestamp of the latest Vision packet.
    double tSent;

    /// Capture timestamp of the previous frame.
    double tCapturePre;

    /// FPS value (optional helper).
    double fps;

    /// Mutex for protecting ourRobot vector.
    mutex ourRobotMutex;

    /// Mutex for protecting enemyRobot vector.
    mutex enemyRobotMutex;

    /// Flag indicating whether a new world update has been received.
    bool isNewWorld = false;

private:
    /**
     * @brief Main loop for receiving Vision packets.
     */
    void receiveLoop(); 

    /// YAML configuration reference.
    const YAML::Node& conf;

    /// Boost.Asio IO context.
    boost::asio::io_context ioContext_;

    /// UDP socket for Vision data.
    boost::asio::ip::udp::socket socket_;

    /// Endpoint for receiving packets.
    boost::asio::ip::udp::endpoint endpoint_;

    /// Receiving thread.
    thread recvThread_;

    /// Running flag for receiver loop.
    atomic<bool> running_;

    /// Our team robots (max 16).
    vector<Robot> ourRobot;

    /// Enemy team robots (max 16).
    vector<Robot> enemyRobot;

    /// FPS counter.
    FpsCounter fpsCounter;

    /// Previous frame number for duplicate detection.
    uint32_t preFrameNumber = 0;
};