// receiver.h

#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include "../models/robot.h"
#include "mocSim_Commands.pb.h"
#include "ssl_vision_wrapper.pb.h"
#include "ssl_vision_detection.pb.h"

class Receiver {
public:
    Receiver();
    ~Receiver();

    void start();
    void stop();

    SSL_WrapperPacket getLatestPacket();

    Robot* getBlueRobots();
    Robot* getYellowRobots();

    double tCapture;
    double tSent;
    double tCapturePre;
    double fps;

private:
    void receiveLoop(); 

    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    std::thread recvThread_;
    std::atomic<bool> running_;

    Robot blueRobot[16];
    Robot yellowRobot[16];

    std::mutex blueRobotMutex;
    std::mutex yellowRobotMutex;
};
