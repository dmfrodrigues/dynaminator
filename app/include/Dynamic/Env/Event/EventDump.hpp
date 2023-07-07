#pragma once

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/SUMOAdapter.hpp"

namespace Dynamic::Env {
class EventDump: public Event {
    SUMO::NetState             &netState;
    const Dynamic::SUMOAdapter &adapter;

    const bool closeAfterDump;

   public:
    EventDump(Time t, SUMO::NetState &netState, const Dynamic::SUMOAdapter &adapter, bool closeAfterDump = false);

    virtual void process(Env &env);
};
}  // namespace Dynamic::Env
