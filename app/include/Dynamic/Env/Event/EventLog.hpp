#pragma once

#include <chrono>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/Event.hpp"

namespace Dynamic::Env {

class EventLog: public Event {
    Time                                           tStartSim, tEndSim;
    std::chrono::high_resolution_clock::time_point tStart;
    Log::ProgressLogger                           &logger;

   public:
    EventLog(
        Time                                           t,
        Time                                           tStartSim,
        Time                                           tEndSim,
        std::chrono::high_resolution_clock::time_point tStart,
        Log::ProgressLogger                           &logger
    );

    virtual void process(Env &env) const;
};

}  // namespace Dynamic::Env
