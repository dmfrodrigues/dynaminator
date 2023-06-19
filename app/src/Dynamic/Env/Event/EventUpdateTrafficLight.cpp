#include "Dynamic/Env/Event/EventUpdateTrafficLight.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/TrafficLight.hpp"

using namespace std;
using namespace Dynamic::Env;

EventUpdateTrafficLight::EventUpdateTrafficLight(
    Time                       t,
    TrafficLight              &trafficLight,
    const TrafficLight::Phase &phase
):
    Event(t), trafficLight(trafficLight), phase(phase) {}

void EventUpdateTrafficLight::process(Env &env) const {
    trafficLight.currentPhase = &phase;

    const TrafficLight::Phase &next = phase.next();

    env.pushEvent(make_shared<EventUpdateTrafficLight>(
        getTime() + phase.duration,
        trafficLight,
        next
    ));
}
