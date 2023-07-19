#include "Dynamic/Env/Event/EventUpdateTrafficLight.hpp"

#include <functional>
#include <stdexcept>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Event/EventPopQueue.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/Position.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "utils/alg.hpp"

using namespace std;
using namespace Dynamic::Env;

EventUpdateTrafficLight::EventUpdateTrafficLight(
    Time                       t,
    TrafficLight              &trafficLight,
    const TrafficLight::Phase &phase
):
    Event(t), trafficLight(trafficLight), phase(phase) {}

void EventUpdateTrafficLight::process(Env &env) {
    trafficLight.currentPhase = &phase;

    const TrafficLight::Phase &next = phase.next();

    env.pushEvent(make_shared<EventUpdateTrafficLight>(
        getTime() + phase.duration,
        trafficLight,
        next
    ));

    set<reference_wrapper<Lane>, less<Lane>> lanes;
    for(const Connection &connection: trafficLight.connections) {
        if(connection.isRed()) continue;
        lanes.insert(connection.fromLane);
    }

    vector<reference_wrapper<Connection>> connections;
    for(const Lane &lane: lanes) {
        if(lane.stopped.empty()) continue;

        Connection &connection = lane.stopped.front().second->connection;

        if(connection.isRed()) continue;

        connections.push_back(connection);
    }

    try {
        utils::alg::topological_sort(
            connections,
            [](const Connection &lhs, const Connection &rhs) -> bool {
                return rhs.yieldsTo(lhs);
            }
        );
    } catch(const std::invalid_argument &e) {
        stringstream ss;
        ss << "e.what(): " << e.what() << "; "
           << "exception happened in traffic light " << trafficLight.id << " "
           << " with connections";
        for(const Connection &connection: trafficLight.connections) {
            ss << " "
               << connection.fromLane.idAsString() << "→"
               << connection.toLane.idAsString();
        }
        throw std::invalid_argument(ss.str());
    }

    for(Connection &connection: connections) {
        env.pushEvent(make_shared<EventPopQueue>(
            env.getTime(),
            connection.fromLane
        ));
    }
}
