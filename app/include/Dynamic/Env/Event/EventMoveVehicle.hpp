#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

class EventMoveVehicle: public Event {
    Vehicle &vehicle;

   public:
    EventMoveVehicle(Time t, Vehicle &vehicle);

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
