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
    endpoint_(boost::asio::ip::address::from_string("127.0.0.1"), 20011) {
    running_ = true;
    socket_.open(boost::asio::ip::udp::v4()); // Open the UDP socket
    timerThread_ = std::thread(&Sender::runTimer, this);
}

Sender::~Sender() {
    running_ = false;
    if (timerThread_.joinable()) {
        timerThread_.join();
    }
    socket_.close(); // Close the socket when done
}

void Sender::send() {
    mocSim_Commands commands;
    commands.set_timestamp(1234567890);
    commands.set_isteamyellow(false);

    mocSim_Robot_Command command;
    command.set_id(0);
    command.set_kickspeedx(1.0);
    command.set_kickspeedz(1.0);
    command.set_veltangent(1.0);
    command.set_velnormal(1.0);
    command.set_velangular(1.0);
    command.set_spinner(1.0);
    command.set_wheel1(0.0);
    command.set_wheel2(0.0);
    command.set_wheel3(0.0);
    command.set_wheel4(0.0);

    // Make sure all required fields are set
    command.set_wheelsspeed(1.0);  // Added missing required field

    commands.add_robot_commands()->CopyFrom(command);

    std::string serializedData;
    if (!commands.SerializeToString(&serializedData)) {
        std::cerr << "Failed to serialize command." << std::endl;
        return;
    }

    // Send the serialized data using the UDP socket
    socket_.send_to(boost::asio::buffer(serializedData), endpoint_);

    std::cout << "Command sent at timestamp: " << commands.timestamp() << std::endl;
}

void Sender::runTimer() {
    const auto frameDuration = std::chrono::milliseconds(1000 / 60); // 60 FPS

    while (running_) {
        auto startTime = std::chrono::high_resolution_clock::now();
        send();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        if (elapsed < frameDuration) {
            std::this_thread::sleep_for(frameDuration - elapsed);
        }
    }
}
