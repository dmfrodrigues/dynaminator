#pragma once

#include <map>
#include <vector>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic::Env {
struct TrafficLight {
   public:
    typedef long ID;

    ID id;

    Time offset;

    struct Phase {
        enum class State {
            RED,
            YELLOW,
            GREEN,
        };

        Time duration;

        std::vector<State> state;
    };

    std::map<Time, Phase> phases;

    Phase &addPhase(Time time, Time duration, std::vector<Phase::State> state);

    TrafficLight(ID id, Time offset);
};
}  // namespace Dynamic::Env
