#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {

/**
 * @brief Try to spawn vehicle.
 *
 * This event instructs the environment to try to spawn a vehicle at time
 * `t`. The vehicle is spawned only if the edge has available space. If not,
 * then a new EventTrySpawnVehicle is scheduled for a later time at which
 * spawning should be retried.
 */
class EventTrySpawnVehicle: public Event {
    Dynamic::Vehicle vehicle;

   public:
    EventTrySpawnVehicle(Time t, const Dynamic::Vehicle &vehicle);

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
