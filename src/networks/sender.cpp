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

void Sender::send(bool is_yellow, double vel, double angle, double orientation) {
    mocSim_Packet packet;
    
    mocSim_Commands commands;
    commands.set_timestamp(1234567890);
    commands.set_isteamyellow(is_yellow);
    for (int i = 0; i < 1; i++) {
        auto* command = commands.add_robot_commands();
        command->set_id(i);
        command->set_kickspeedx(0.0);
        command->set_kickspeedz(0.0);
        
        float vel_x = vel * cos(angle);
        float vel_y = vel * sin(angle);
        float rel_vel_x =  vel_x * cos(orientation) - vel_y * sin(orientation);
        float rel_vel_y =  vel_x * sin(orientation) + vel_y * cos(orientation);

        command->set_veltangent(vel/1000);
        // std::cout << "Sending velocity: " << vel << ", angle: " << angle << std::endl;
        command->set_velnormal(0);
        command->set_velangular(angle);
        command->set_spinner(false);
        command->set_wheelsspeed(false);
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