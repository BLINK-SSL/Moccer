// receiver.cpp

#include "receiver.h"


Receiver::Receiver()
    : socket_(ioContext_),
      endpoint_(boost::asio::ip::make_address("224.5.23.2"), 10694),
      running_(false) {

    socket_.open(boost::asio::ip::udp::v4());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(endpoint_);
    
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::make_address("224.5.23.2")));
    
    for (int i = 0; i < 16; ++i) {
        blueRobot[i] = Robot();
        yellowRobot[i] = Robot();
    }
}

Receiver::~Receiver() {
    stop();
}

void Receiver::start() {
    running_ = true;
    recvThread_ = std::thread(&Receiver::receiveLoop, this);
}

void Receiver::stop() {
    running_ = false;
    if (recvThread_.joinable()) {
        recvThread_.join();
    }
}

void Receiver::receiveLoop() {
    while (running_) {
        try {
            char recvBuf[2048];
            boost::asio::ip::udp::endpoint senderEndpoint;
            size_t len = socket_.receive_from(boost::asio::buffer(recvBuf), senderEndpoint);

            SSL_WrapperPacket packet;
            if (packet.ParseFromArray(recvBuf, static_cast<int>(len))) {
                SSL_DetectionFrame detection = packet.detection();
                tCapture = detection.t_capture();
                tSent = detection.t_sent();
                if (detection.t_capture() - tCapturePre != 0) {
                    fps = 1.0 / (detection.t_capture() - tCapturePre);
                }
                
                tCapturePre = detection.t_capture();
                {
                    std::lock_guard<std::mutex> lock(blueRobotMutex);
                    for (const auto& robot : detection.robots_blue()) {
                        blueRobot[robot.robot_id()].update(robot);
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(yellowRobotMutex);
                    for (const auto& robot : detection.robots_yellow()) {
                        yellowRobot[robot.robot_id()].update(robot);
                    }
                }
            } else {
                std::cerr << "Failed to parse packet" << std::endl;
            }
        } catch (std::exception& e) {
            std::cerr << "Receive error: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

Robot* Receiver::getBlueRobots() {
    std::lock_guard<std::mutex> lock(blueRobotMutex);
    return blueRobot;
}

Robot* Receiver::getYellowRobots() {
    std::lock_guard<std::mutex> lock(yellowRobotMutex);
    return yellowRobot;
}