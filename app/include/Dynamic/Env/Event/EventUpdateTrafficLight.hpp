#pragma once

#include "Dynamic/Env/Event/Event.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "Dynamic/Vehicle.hpp"

namespace Dynamic::Env {

class EventUpdateTrafficLight: public Event {
    TrafficLight              &trafficLight;
    const TrafficLight::Phase &phase;

   public:
    EventUpdateTrafficLight(
        Time                       t,
        TrafficLight              &trafficLight,
        const TrafficLight::Phase &phase
    );

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
