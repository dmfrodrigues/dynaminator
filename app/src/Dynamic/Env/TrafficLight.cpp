#include "Dynamic/Env/TrafficLight.hpp"

#include <cmath>
#include <iostream>

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

TrafficLight::Phase::Phase(TrafficLight &tl_, Time t_, Time duration_, vector<State> state_):
    tl(tl_), t(t_), duration(duration_), state(state_) {}

TrafficLight::Phase &TrafficLight::Phase::next() {
    auto it = tl.phases.upper_bound(t);
    return (it == tl.phases.end() ? tl.phases.begin() : it)->second;
}

const TrafficLight::Phase &TrafficLight::Phase::next() const {
    auto it = tl.phases.upper_bound(t);
    return (it == tl.phases.end() ? tl.phases.begin() : it)->second;
}

TrafficLight::Phase &TrafficLight::addPhase(Time time, Time duration, vector<Phase::State> state) {
    return phases.emplace(time, Phase{*this, time, duration, state}).first->second;
}

TrafficLight::TrafficLight(ID id_, Time offset_):
    id(id_), offset(offset_) {}

pair<const TrafficLight::Phase &, Time> TrafficLight::getPhase(Time t_) const {
    Time t = t_;

    t -= offset;

    Time d = duration();

    t = fmod(t, d);
    t = (t < 0 ? t + d : t);

    auto it = phases.upper_bound(t);
    --it;

    const Phase &phase = it->second;

    Time dt = t - phase.t;

    Time tStart_ = t_ - dt;

    return {phase, tStart_};
}

Time TrafficLight::duration() const {
    Time ret = 0.0;
    for(const auto &[_, phase]: phases)
        ret += phase.duration;
    return ret;
}
