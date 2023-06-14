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
            edge.lanes.size(),
            edge.speed()
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

    for(const SUMO::Network::Edge &from: sumo.network.getEdges()) {
        if(from.function == SUMO::Network::Edge::Function::INTERNAL) continue;

        for(const SUMO::Network::Edge *toPtr: from.getOutgoing()) {
            const SUMO::Network::Edge &to = *toPtr;

            if(to.function == SUMO::Network::Edge::Function::INTERNAL) continue;

            addConnection(sumo, from, to);
        }
    }
}

// clang-format off
void Loader<
    const SUMO::NetworkTAZs &
>::addConnection(
    const SUMO::NetworkTAZs &sumo,
    const SUMO::Network::Edge &fromSUMO,
    const SUMO::Network::Edge &toSUMO
) {
    auto fromToConnections = sumo.network.getConnections(fromSUMO, toSUMO);

    if(fromToConnections.empty()) return;

    Connection::ID connectionID = nextConnectionID++;

    Edge::ID fromID = adapter.toEdge(fromSUMO.id);
    Edge::ID toID = adapter.toEdge(toSUMO.id);

    const Edge &from = env->getEdge(fromID);
    const Edge &to   = env->getEdge(toID);

    env->addConnection(
        connectionID,
        from,
        to
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
