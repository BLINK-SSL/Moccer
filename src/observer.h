#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>

#include "networks/sender.h"
#include "networks/receiver.h"
#include "finder/Dstar.h"

class Observer {
    public:
        Observer();
        ~Observer();

        void update();
        void waitForReceiver();

        Sender sender;
        Receiver receiver;
        Dstar dstar;

    private:
        Robot* blueRobots;
        Robot* yellowRobots;
        Robot* preBlueRobots;
        Robot* preYellowRobots;

};

#endif // OBSERVER_H