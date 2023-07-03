#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {

/**
 * @brief Spawn vehicle.
 *
 * This event instructs the environment to spawn a vehicle at time
 * `t`. The vehicle is spawned only if the edge has available space. If not,
 * then it is enqueued to a special queue.
 */
class EventSpawnVehicle: public Event {
    Dynamic::Vehicle vehicle;

   public:
    EventSpawnVehicle(Time t, const Dynamic::Vehicle &vehicle);

    virtual void process(Env &env);
};

}  // namespace Dynamic::Env
