#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>

#include "networks/sender.h"
#include "networks/receiver.h"

class Observer {
    public:
        Observer();
    private:
        Sender sender;
        Receiver receiver;
};

#endif // OBSERVER_H