#pragma once

#include "Dynamic/Env/Event/EventMoveVehicle.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

/**
 * @brief Update vehicle position and speed.
 *
 * This event takes the latest vehicle information, and applies the
 * respective movement equations to update the vehicle position and speed
 * according to the current time as indicated by the Environment.
 *
 * This step is uniquely linked to the environment model, since the movement
 * equations are part of the environment model.
 */
class EventUpdateVehicle: public EventMoveVehicle {
    Vehicle &vehicle;

   public:
    EventUpdateVehicle(Time t, Vehicle &vehicle);

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
