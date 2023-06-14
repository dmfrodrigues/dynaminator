#pragma once

#include <memory>
#include <vector>

#include "Dynamic/Dynamic.hpp"
#include "Dynamic/Env/Event/Event.hpp"

namespace Dynamic::Env {

class EventComposite: public Event {
    std::vector<std::shared_ptr<Event>> events;

   public:
    EventComposite(Time t);
    EventComposite(Time t, std::initializer_list<std::shared_ptr<Event>> initList);
    void         addEvent(std::shared_ptr<Event> event);
    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
