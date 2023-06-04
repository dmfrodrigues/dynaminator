#pragma once

#include <unordered_map>
#include <utility>

#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"
#include "Dynamic/Environment.hpp"

namespace Dynamic {
class SUMOAdapter: private Static::SUMOAdapter {
   public:
    SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter);

    std::pair<Environment::Node, Environment::Node> addSumoTAZ(const SUMO::TAZ::ID &a);
    // clang-format off
    std::pair<
        Environment::Edge::ID, std::pair<
            Environment::Node,
            Environment::Node
        >
    > addSumoEdge(const SUMO::Network::Edge::ID &a);
    // clang-format on
    Environment::Edge::ID addEdge();
    bool isEdge(const SUMO::Network::Edge::ID &a) const;

    const std::pair<Environment::Node, Environment::Node> toTAZNode(const SUMO::TAZ::ID &a) const;
    const SUMO::TAZ::ID toSumoTAZ(const Environment::Node &a) const;

    const Environment::Edge::ID toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID toSumoEdge(const Environment::Edge::ID &a) const;
    const std::pair<Environment::Node, Environment::Node> toNodes(const SUMO::Network::Edge::ID &a) const;

    bool isSumoEdge(const Environment::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;

    SUMO::Network::Edge::ID fromNodeToSumoEdge(const Environment::Node &a) const;

    void clear();
};
}  // namespace Static
