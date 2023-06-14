#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/Vehicle.hpp"

namespace Dynamic::Env {

/**
 * @brief Force vehicle to pick connection in current edge.
 *
 * This event is triggered when a vehicle reaches the end of an edge/lane,
 * and must pick a connection to continue its path.
 *
 * This is where different vehicle routing policies can affect the final
 * result. When processing this event, a policy that is external to the
 * environment is applied.
 *
 * This event is equivalent to forcing the vehicle driver to choose what
 * they want to do. This is also the part where vehicle drivers can express
 * their objectives and path preferences.
 */
class EventPickConnection: public Event {
    Vehicle &vehicle;

   public:
    EventPickConnection(Time t, Vehicle &vehicle);

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
