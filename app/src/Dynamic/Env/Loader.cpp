#include "Dynamic/Env/Loader.hpp"

#include "Dynamic/Env/Env.hpp"

using namespace std;
using namespace Dynamic::Env;

// clang-format off
Env Loader<
    const SUMO::NetworkTAZs &
>::load(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    Env ret;

    env = &ret;

    addEdges(sumo);

    addConnections(sumo);

    addTAZs(sumo);

    return ret;
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &
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
            edge.lanes.size()
        );
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &
>::addConnections(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    auto connections = sumo.network.getConnections();
    for(const auto &[fromID, connectionsFrom]: connections) {
        for(const auto &[toID, connectionsFromTo]: connectionsFrom) {
            for(const SUMO::Network::Connection *connection: connectionsFromTo) {
                addConnection(sumo, *connection);
            }
        }
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &
>::addConnection(
    const SUMO::NetworkTAZs &sumo,
    const SUMO::Network::Connection &connection
) {
    Connection::ID connectionID = nextConnectionID++;

    if(
        connection.from.function == SUMO::Network::Edge::Function::INTERNAL ||
        connection.to.function == SUMO::Network::Edge::Function::INTERNAL
    ) return;

    Edge::ID fromID = adapter.toEdge(connection.from.id);
    Edge::ID toID   = adapter.toEdge(connection.to.id);

    const Edge &from = env->getEdge(fromID);
    const Edge &to   = env->getEdge(toID);

    const Edge::Lane &fromLane = from.lanes.at(connection.fromLaneIndex);
    const Edge::Lane &toLane   = to.lanes.at(connection.toLaneIndex);

    env->addConnection(
        connectionID,
        fromLane,
        toLane
    );
}

void Loader<
    const SUMO::NetworkTAZs &>::addTAZs(const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    for(const auto &[id, taz]: sumo.tazs) {
        adapter.addSumoTAZ(
            taz.id,
            taz.sources,
            taz.sinks
        );
    }
}
