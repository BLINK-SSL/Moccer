#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>

#include "networks/sender.h"
#include "networks/receiver.h"

class Observer {
    public:
        Observer();
        ~Observer();

        void update();
        Sender sender;
        Receiver receiver;


    // private:
        Robot* blueRobots;
        Robot* yellowRobots;

};

#endif // OBSERVER_H