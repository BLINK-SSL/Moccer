// receiver.h

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

class FpsCounter {
public:
    FpsCounter() : frameCounter(0), startTime(std::chrono::high_resolution_clock::now()) {}

    void start() {
        running_ = true;
        fpsThread = std::thread(&FpsCounter::calculateFps, this);
    }

    void stop() {
        running_ = false;
        if (fpsThread.joinable()) {
            fpsThread.join();
        }
    }

    void calculateFps() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (frameCounter > 0) {
                std::lock_guard<std::mutex> lock(secPerFrameMutex);
                secPerFrame = 1.0 / (frameCounter);
                frameCounter = 0;
            }
            std::cout << "FPS: " << 1.0 / secPerFrame << std::endl;
        }
    }

    std::mutex secPerFrameMutex;
    int frameCounter;
    double secPerFrame;

    double getSecPerFrame() {
        std::lock_guard<std::mutex> lock(secPerFrameMutex);
        return secPerFrame;
    }

private:
    
    double fps;
    std::chrono::high_resolution_clock::time_point startTime;

    std::thread fpsThread;
    std::atomic<bool> running_;
};


class Receiver {
public:
    Receiver(const YAML::Node& config);
    ~Receiver();

    void start();
    void stop();

    SSL_WrapperPacket getLatestPacket();

    std::vector<Robot> getOurRobots();
    std::vector<Robot> getEnemyRobots();

    double tCapture;
    double tSent;
    double tCapturePre;
    double fps;

    std::mutex ourRobotMutex;
    std::mutex enemyRobotMutex;
    
    bool isNewWorld = false;

private:
    void receiveLoop(); 

    const YAML::Node& conf;

    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    std::thread recvThread_;
    std::atomic<bool> running_;

    std::vector<Robot> ourRobot;
    std::vector<Robot> enemyRobot;

    FpsCounter fpsCounter;
    uint32_t preFrameNumber = 0;
};

