#pragma once

#include "Dynamic/SUMOAdapter.hpp"

namespace Dynamic::Env {

class Env;

template<typename T, typename... Args>
class Loader {
   public:
    Env load(T arg1, Args... arg2);
};

template<>
class Loader<const SUMO::NetworkTAZs &> {
    Env *env;

    void addEdges(const SUMO::NetworkTAZs &sumo);
    void addConnections(const SUMO::NetworkTAZs &sumo);
    void addTAZs(const SUMO::NetworkTAZs &sumo);

    Connection::ID nextConnectionID = 1;

    virtual void addConnection(
        const SUMO::NetworkTAZs   &sumo,
        const SUMO::Network::Edge &from,
        const SUMO::Network::Edge &to
    );

   public:
    SUMOAdapter adapter;

    Env load(const SUMO::NetworkTAZs &sumo);
};
}  // namespace Dynamic::Env
