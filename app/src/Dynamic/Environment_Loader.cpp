#include "Dynamic/Environment_Loader.hpp"

using namespace std;
using namespace Dynamic;

// clang-format off
Environment *Environment::Loader<
    const SUMO::NetworkTAZs &
>::load(
    const SUMO::NetworkTAZs &sumo
) {
    // clang-format on

    env = new Environment();

    // TODO: implement
    addEdges(sumo);

    addConnections(sumo);

    addDeadEnds(sumo);

    addTAZs(sumo);

    return env;
}

// clang-format off
void Environment::Loader<
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

//  clang-format off
void Environment::Loader<
    const SUMO::NetworkTAZs &>::addConnections(const SUMO::NetworkTAZs &sumo
) {
    // clang-format on
}

void Environment::Loader<
    const SUMO::NetworkTAZs &>::addDeadEnds(const SUMO::NetworkTAZs &sumo
) {
    // clang-format on
}

void Environment::Loader<
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
