#include "Dynamic/Env/Loader.hpp"

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "Dynamic/Env/TrafficLight.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

using namespace std;
using namespace Dynamic::Env;

// clang-format off
Env Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::load(
    const SUMO::NetworkTAZs &sumo,
    RewardFunction          &rewardFunction
) {
    // clang-format on

    Env ret(rewardFunction);

    env = &ret;

    addTrafficLights(sumo);

    addEdges(sumo);

    addConnections(sumo);

    addTAZs(sumo);

    return ret;
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addTrafficLights(
    const SUMO::NetworkTAZs &sumo
) {
    for(const auto &[sumoTLID, sumoTL]: sumo.network.getTrafficLights()){
        TrafficLight &tl = env->addTrafficLight(
            adapter.addSumoTL(sumoTLID),
            sumoTL.offset
        );
        for(const auto &[pTime, p]: sumoTL.phases){
            vector<TrafficLight::Phase::State> state;
            for(const SUMO::Network::TrafficLightLogic::Phase::State &s: p.state){
                switch(s){
                    case SUMO::Network::TrafficLightLogic::Phase::State::RED:
                        state.push_back(TrafficLight::Phase::State::RED);
                        break;
                    case SUMO::Network::TrafficLightLogic::Phase::State::YELLOW_STOP:
                        state.push_back(TrafficLight::Phase::State::YELLOW);
                        break;
                    default:
                        state.push_back(TrafficLight::Phase::State::GREEN);
                        break;
                }
            }

            tl.addPhase(
                pTime,
                p.duration,
                state
            );
        }
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addEdges(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    const vector<SUMO::Network::Edge> &edges = sumo.network.getEdges();
    for(const SUMO::Network::Edge &edge: edges) {
        if(edge.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        const auto     &p   = adapter.addSumoEdge(edge.id);
        const Edge::ID &eid = p.first;
        Node            u = p.second.first, v = p.second.second;

        env->addEdge(
            eid,
            u,
            v,
            edge.length(),
            edge.speed(),
            edge.priority,
            edge.lanes.size()
        );
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addConnections(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    auto connections = sumo.network.getConnections();
    for(const auto &[fromID, connectionsFrom]: connections) {
        for(const auto &[toID, connectionsFromTo]: connectionsFrom) {
            for(const SUMO::Network::Connection &connection: connectionsFromTo) {
                addConnection(sumo, connection);
            }
        }
    }

    for(const auto &[fromID, connectionsFrom]: connections) {
        for(const auto &[toID, connectionsFromTo]: connectionsFrom) {
            for(const SUMO::Network::Connection &connection: connectionsFromTo) {
                addConflicts(sumo, connection);
            }
        }
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addConnection(
    const SUMO::NetworkTAZs &sumo,
    const SUMO::Network::Connection &connection
) {
    // clang-format on
    Connection::ID connectionID = nextConnectionID++;

    // clang-format off
    if(
        connection.from.function == SUMO::Network::Edge::Function::INTERNAL ||
        connection.to.function == SUMO::Network::Edge::Function::INTERNAL
    ) return;
    // clang-format on

    Edge::ID fromID = adapter.toEdge(connection.from.id);
    Edge::ID toID   = adapter.toEdge(connection.to.id);

    Edge &from = env->getEdge(fromID);
    Edge &to   = env->getEdge(toID);

    Lane &fromLane = from.lanes.at(connection.fromLaneIndex);
    Lane &toLane   = to.lanes.at(connection.toLaneIndex);

    Connection &conn = env->addConnection(
        connectionID,
        fromLane,
        toLane
    );

    if(connection.tl) {
        conn.trafficLight = env->getTrafficLight(adapter.toTL(connection.tl.value().get().id));
        conn.tlLinkIndex  = connection.linkIndex;

        conn.trafficLight.value().get().connections.emplace_back(conn);
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addConflicts(
    const SUMO::NetworkTAZs &sumo,
    const SUMO::Network::Connection &connection
) {
    // clang-format on

    // clang-format off
    if(
        connection.from.function == SUMO::Network::Edge::Function::INTERNAL ||
        connection.to.function == SUMO::Network::Edge::Function::INTERNAL
    ) return;
    // clang-format on

    Edge::ID    fromID        = adapter.toEdge(connection.from.id);
    Lane::Index fromLaneIndex = connection.fromLaneIndex;

    Edge::ID    toID        = adapter.toEdge(connection.to.id);
    Lane::Index toLaneIndex = connection.toLaneIndex;

    Lane &fromLane = env->getEdge(fromID).lanes.at(fromLaneIndex);
    Lane &toLane   = env->getEdge(toID).lanes.at(toLaneIndex);

    Connection &envConnection = fromLane.getOutgoingConnection(toLane);

    for(const SUMO::Network::Connection &otherConnection: connection.getRequest().getResponse()) {
        // clang-format off
        if(
            otherConnection.from.function == SUMO::Network::Edge::Function::INTERNAL ||
            otherConnection.to.function == SUMO::Network::Edge::Function::INTERNAL
        ) continue;
        // clang-format on

        Edge::ID    otherFromID        = adapter.toEdge(otherConnection.from.id);
        Lane::Index otherFromLaneIndex = otherConnection.fromLaneIndex;

        Edge::ID    otherToID        = adapter.toEdge(otherConnection.to.id);
        Lane::Index otherToLaneIndex = otherConnection.toLaneIndex;

        Lane &otherFromLane = env->getEdge(otherFromID).lanes.at(otherFromLaneIndex);
        Lane &otherToLane   = env->getEdge(otherToID).lanes.at(otherToLaneIndex);

        Connection &otherEnvConnection = otherFromLane.getOutgoingConnection(otherToLane);

        envConnection.addMoreImportant(otherEnvConnection);
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &,
    Dynamic::RewardFunction &
>::addTAZs(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    for(const auto &[id, taz]: sumo.tazs) {
        TAZ::ID envTAZID = adapter.addSumoTAZ(
            taz.id,
            taz.sources,
            taz.sinks
        );

        TAZ &envTAZ = env->addTAZ(envTAZID);

        for(const SUMO::TAZ::Source &source: taz.sources) {
            envTAZ.sources.insert(env->getEdge(adapter.toEdge(source.id)));
        }

        for(const SUMO::TAZ::Sink &sink: taz.sinks) {
            envTAZ.sinks.insert(env->getEdge(adapter.toEdge(sink.id)));
        }
    }
}
