#pragma once

#include "Dynamic/Environment.hpp"
#include "Dynamic/SUMOAdapter.hpp"

namespace Dynamic {
template<>
class Environment::Loader<const SUMO::NetworkTAZs &> {
    Environment *env;

    void addEdges(const SUMO::NetworkTAZs &sumo);
    void addConnections(const SUMO::NetworkTAZs &sumo);
    void addDeadEnds(const SUMO::NetworkTAZs &sumo);
    void addTAZs(const SUMO::NetworkTAZs &sumo);

    Environment::Connection::ID nextConnectionID = 1;

    virtual void addConnection(
        const SUMO::NetworkTAZs &sumo,
        const SUMO::Network::Edge &from,
        const SUMO::Network::Edge &to
    );

   public:
    SUMOAdapter adapter;

    Environment *load(const SUMO::NetworkTAZs &sumo);
};
}  // namespace Dynamic
