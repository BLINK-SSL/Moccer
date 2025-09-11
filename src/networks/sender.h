#ifndef SENDER_H
#define SENDER_H

#include <iostream>
#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "mocSim_Commands.pb.h"
#include "mocSim_Packet.pb.h"

#include "../models/cmd.h"

using namespace std;

class Sender {
public:
    Sender(const YAML::Node& config);
    ~Sender();
    void send(bool is_yellow, RobotCmd* cmds);

private:
    const YAML::Node& conf;

    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    float angle;
};

#endif // SENDER_H
