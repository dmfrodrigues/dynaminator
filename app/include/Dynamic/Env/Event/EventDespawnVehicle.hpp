#pragma once

#include "Dynamic/Env/Env.hpp"

namespace Dynamic::Env {
class EventDespawnVehicle: public Event {
    Vehicle::ID vehicleID;

   public:
    EventDespawnVehicle(Time t, Vehicle::ID vehicleID);

    virtual void process(Env &env);
};
}  // namespace Dynamic::Env
