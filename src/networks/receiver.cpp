// receiver.cpp

#include "receiver.h"


Receiver::Receiver(const YAML::Node& config)
    : socket_(ioContext_),
      endpoint_(boost::asio::ip::make_address(config["Network"]["Vision"]["Address"].as<string>()), config["Network"]["Vision"]["Port"].as<uint16_t>()),
      conf(config),
      running_(false) {

    socket_.open(boost::asio::ip::udp::v4());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(endpoint_);
    
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::make_address(config["Network"]["Vision"]["Address"].as<string>())));
    
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
    recvThread_ = thread(&Receiver::receiveLoop, this);
}

void Receiver::stop() {
    running_ = false;
    if (recvThread_.joinable()) {
        recvThread_.join();
    }
    fpsCounter.stop();
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

                uint32_t frame_number = detection.frame_number();
                if (frame_number != preFrameNumber) {
                    fpsCounter.frameCounter++;
                    preFrameNumber = frame_number;
                    isNewWorld = true;
                }

                {
                    lock_guard<mutex> lock(ourRobotMutex);
                    if (conf["General"]["Color"].as<string>() == "blue") {
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
                    lock_guard<mutex> lock(enemyRobotMutex);
                    if (conf["General"]["Color"].as<string>() == "blue") {
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
                cerr << "Failed to parse packet" << endl;
            }
        } catch (exception& e) {
            cerr << "Receive error: " << e.what() << endl;
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

vector<Robot> Receiver::getOurRobots() {
    lock_guard<mutex> lock(ourRobotMutex);
    return ourRobot;
}

vector<Robot> Receiver::getEnemyRobots() {
    lock_guard<mutex> lock(enemyRobotMutex);
    return enemyRobot;
}

