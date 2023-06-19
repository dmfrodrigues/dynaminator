#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

class EventPopQueue: public Event {
    Lane &lane;

   public:
    EventPopQueue(Time t, Lane &lane);

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
