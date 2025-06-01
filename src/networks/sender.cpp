#include "sender.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include "mocSim_Commands.pb.h"
#include "mocSim_Packet.pb.h"

Sender::Sender() :
    ioContext_(),
    socket_(ioContext_),
    endpoint_(boost::asio::ip::make_address("127.0.0.1"), 20694) {
    socket_.open(boost::asio::ip::udp::v4());
}

Sender::~Sender() {
}

void Sender::send(bool is_yellow) {
    mocSim_Packet packet;
    
    mocSim_Commands commands;
    commands.set_timestamp(1234567890);
    commands.set_isteamyellow(is_yellow);
    angle += 0.01;
    if (angle > 3.14159) {
        angle = -3.14159;
    }
    for (int i = 0; i < 2; i++) {
        auto* command = commands.add_robot_commands();
        command->set_id(i);
        command->set_kickspeedx(0.0);
        command->set_kickspeedz(0.0);
        command->set_veltangent(1);
        command->set_velnormal(1);
        command->set_velangular(0);
        command->set_spinner(true);
        command->set_wheelsspeed(0.0);
        // command->set_wheel1(0.0);
        // command->set_wheel2(0.0);
        // command->set_wheel3(0.0);
        // command->set_wheel4(0.0);
    }
    
    packet.mutable_commands()->CopyFrom(commands);

    std::string serializedData;
    if (!packet.SerializeToString(&serializedData)) {
        std::cerr << "Failed to serialize command." << std::endl;
        return;
    }

    // Send the serialized data using the UDP socket
    socket_.send_to(boost::asio::buffer(serializedData), endpoint_);
}