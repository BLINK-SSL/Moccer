// receiver.cpp

#include "receiver.h"


Receiver::Receiver(const YAML::Node& config)
    : socket_(ioContext_),
      endpoint_(boost::asio::ip::make_address(config["Network"]["Vision"]["Address"].as<std::string>()), config["Network"]["Vision"]["Port"].as<uint16_t>()),
      conf(config),
      running_(false) {

    socket_.open(boost::asio::ip::udp::v4());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(endpoint_);
    
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::make_address(config["Network"]["Vision"]["Address"].as<std::string>())));
    
    for (int i = 0; i < 16; ++i) {
        ourRobot.push_back(Robot());
        enemyRobot.push_back(Robot());
    }
    fpsCounter.start();
    isNewWorld = false;
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
            
            char buff[2048];
            SSL_WrapperPacket packet;
            if (packet.ParseFromArray(recvBuf, static_cast<int>(len))) {
                SSL_DetectionFrame detection = packet.detection();

                uint32_t frame_number = detection.frame_number();
                if (frame_number != preFrameNumber) {
                    fpsCounter.frameCounter++;
                    preFrameNumber = frame_number;
                    isNewWorld = true;
                }

                {
                    std::lock_guard<std::mutex> lock(ourRobotMutex);
                    if (conf["General"]["Color"].as<std::string>() == "blue") {
                        for (const auto& robot : detection.robots_blue()) {
                            ourRobot[robot.robot_id()].update(robot, fpsCounter.getSecPerFrame());
                        }
                    } else {
                        for (const auto& robot : detection.robots_yellow()) {
                            ourRobot[robot.robot_id()].update(robot, fpsCounter.getSecPerFrame());
                        }
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(enemyRobotMutex);
                    if (conf["General"]["Color"].as<std::string>() == "blue") {
                        for (const auto& robot : detection.robots_yellow()) {
                            enemyRobot[robot.robot_id()].update(robot, fpsCounter.getSecPerFrame());
                        }
                    } else {
                        for (const auto& robot : detection.robots_blue()) {
                            enemyRobot[robot.robot_id()].update(robot, fpsCounter.getSecPerFrame());
                        }
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

std::vector<Robot> Receiver::getOurRobots() {
    // std::lock_guard<std::mutex> lock(ourRobotMutex);
    return ourRobot;
}

std::vector<Robot> Receiver::getEnemyRobots() {
    // std::lock_guard<std::mutex> lock(enemyRobotMutex);
    return enemyRobot;
}
