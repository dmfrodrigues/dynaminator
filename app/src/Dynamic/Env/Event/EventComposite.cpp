#include "Dynamic/Env/Event/EventComposite.hpp"

#include <iostream>

#include "Dynamic/Env/Event/EventPickConnection.hpp"
#include "Dynamic/Env/Event/EventTrySpawnVehicle.hpp"
#include "Dynamic/Env/Event/EventUpdateVehicle.hpp"

using namespace std;
using namespace Dynamic::Env;

EventComposite::EventComposite(Time t_):
    Event(t_) {}
EventComposite::EventComposite(Time t_, initializer_list<shared_ptr<Event>> events_):
    Event(t_), events(events_) {}

void EventComposite::addEvent(shared_ptr<Event> event) { events.push_back(event); }
void EventComposite::process(Env &env) const {
    for(auto &event: events) {
        event->process(env);
    }
}
