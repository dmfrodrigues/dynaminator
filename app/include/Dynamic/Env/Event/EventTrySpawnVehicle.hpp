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

    unsigned attempt = 0;

   public:
    static const unsigned       MAX_ATTEMPTS          = 12;
    constexpr static const Time TIME_BETWEEN_ATTEMPTS = 10.0;

    EventTrySpawnVehicle(Time t, const Dynamic::Vehicle &vehicle, unsigned attempt = 0);

    virtual void process(Env &env);
};

}  // namespace Dynamic::Env
