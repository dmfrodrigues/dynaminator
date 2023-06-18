#include "Dynamic/Env/TrafficLight.hpp"

using namespace std;
using namespace Dynamic::Env;

TrafficLight::Phase &TrafficLight::addPhase(Time time, Time duration, vector<Phase::State> state) {
    return phases.emplace(time, Phase{duration, state}).first->second;
}

TrafficLight::TrafficLight(ID id_, Time offset_):
    id(id_), offset(offset_) {}
