#pragma once

#include "Dynamic/SUMOAdapter.hpp"

namespace Dynamic::Env {

class Env;

template<typename T, typename... Args>
class Loader {
   public:
    std::shared_ptr<Env> load(T arg1, Args... arg2);
};

// clang-format off
template<>
class Loader<
    const SUMO::NetworkTAZs &,
    RewardFunction &
> {
    // clang-format on

    Env *env;

    void addTrafficLights(const SUMO::NetworkTAZs &sumo);
    void addEdges(const SUMO::NetworkTAZs &sumo);
    void addConnections(const SUMO::NetworkTAZs &sumo);
    void addTAZs(const SUMO::NetworkTAZs &sumo);

    Connection::ID nextConnectionID = 1;

    virtual void addConnection(
        const SUMO::NetworkTAZs         &sumo,
        const SUMO::Network::Connection &connection
    );
    virtual void addConflicts(
        const SUMO::NetworkTAZs         &sumo,
        const SUMO::Network::Connection &connection
    );

   public:
    SUMOAdapter adapter;

    std::shared_ptr<Env> load(
        const SUMO::NetworkTAZs &sumo,
        RewardFunction          &rewardFunction
    );
};
}  // namespace Dynamic::Env
