#ifndef SENDER_H
#define SENDER_H

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include "mocSim_Commands.pb.h"
#include "mocSim_Packet.pb.h"

using namespace std;

class Sender {
public:
    Sender();
    ~Sender();
    void send();

private:
    void runTimer();
    bool running_;
    std::thread timerThread_;
    
    // Networking related members
    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;
};

#endif // SENDER_H
