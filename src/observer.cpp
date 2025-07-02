#include "observer.h"

Observer::Observer() : sender(), receiver()
{
    receiver.start();
}

