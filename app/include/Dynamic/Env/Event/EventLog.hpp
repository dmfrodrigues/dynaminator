#pragma once

#include <chrono>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/Event.hpp"

namespace Dynamic::Env {

class EventLog: public Event {
    Time                                  tStartSim, tEndSim;
    std::chrono::steady_clock::time_point tStart;
    Log::ProgressLogger                  &logger;
    Policy::Logger                       &policyLogger;

   public:
    EventLog(
        Time                                  t,
        Time                                  tStartSim,
        Time                                  tEndSim,
        std::chrono::steady_clock::time_point tStart,
        Log::ProgressLogger                  &logger,
        Policy::Logger                       &policyLogger
    );

    virtual void process(Env &env);
};

}  // namespace Dynamic::Env
