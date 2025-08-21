#include "sender.h"

Sender::Sender(const YAML::Node& config) :
    ioContext_(),
    socket_(ioContext_),
    endpoint_(boost::asio::ip::make_address(conf["Network"]["Simulation"]["Address"].as<std::string>()), conf["Network"]["Simulation"]["Port"].as<uint16_t>()),
    conf(config) 
{
    socket_.open(boost::asio::ip::udp::v4());
}

Sender::~Sender() {
}

void Sender::send(bool is_yellow, RobotCmd* cmds) {
    mocSim_Packet packet;
    
    mocSim_Commands commands;
    commands.set_timestamp(1234567890);
    commands.set_isteamyellow(is_yellow);
    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); i++) {
        // std::cout << "vel: " << cmds[i].vel.x() << ", " << cmds[i].vel.y() << ", " << cmds[i].angVel << std::endl;
        auto* command = commands.add_robot_commands();
        command->set_id(cmds[i].id);
        command->set_kickspeedx(cmds[i].kickPow);
        command->set_kickspeedz(cmds[i].chipKickPow);
        command->set_veltangent(cmds[i].vel.x());
        command->set_velnormal(cmds[i].vel.y());
        command->set_velangular(cmds[i].angVel);
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

    socket_.send_to(boost::asio::buffer(serializedData), endpoint_);
}