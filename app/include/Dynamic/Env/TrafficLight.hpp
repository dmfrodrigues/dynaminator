#pragma once

#include <functional>
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

       private:
        TrafficLight &tl;

       public:
        Time t;
        Time duration;

        std::vector<State> state;

        Phase(TrafficLight &tl, Time t, Time duration, std::vector<State> state);

        Phase       &next();
        const Phase &next() const;
    };

    std::map<Time, Phase> phases;

    const Phase *currentPhase;

    Phase &addPhase(Time time, Time duration, std::vector<Phase::State> state);

    TrafficLight(ID id, Time offset);

    std::pair<const Phase &, Time> getPhase(Time t) const;

    Time duration() const;
};
}  // namespace Dynamic::Env
