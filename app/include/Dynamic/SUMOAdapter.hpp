#pragma once

#include <unordered_map>
#include <utility>

#include "Dynamic/Environment.hpp"
#include "Static/SUMOAdapter.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Dynamic {
class SUMOAdapter: private Static::SUMOAdapter {
   private:
    // clang-format off
    std::unordered_map<
        SUMO::TAZ::ID,
        std::pair<
            std::list<SUMO::TAZ::Source>,
            std::list<SUMO::TAZ::Sink>
        >
    > sumoTAZSourcesSinks;
    //  clang-format on

   public:
    SUMOAdapter();
    SUMOAdapter(Static::SUMOAdapter &staticSUMOAdapter);

    void addSumoTAZ(
        const SUMO::TAZ::ID                &a,
        const std::list<SUMO::TAZ::Source> &sources,
        const std::list<SUMO::TAZ::Sink>   &sinks
    );
    // clang-format off
    std::pair<
        Environment::Edge::ID, std::pair<
            Environment::Node,
            Environment::Node
        >
    > addSumoEdge(const SUMO::Network::Edge::ID &a);
    // clang-format on
    Environment::Edge::ID addEdge();
    bool                  isEdge(const SUMO::Network::Edge::ID &a) const;

    // clang-format off
    std::pair<
        std::list<SUMO::TAZ::Source>,
        std::list<SUMO::TAZ::Sink>
    > toTAZEdges(const SUMO::TAZ::ID &a) const;
    // clang-format on
    const std::pair<Environment::Node, Environment::Node> toTAZNode(const SUMO::TAZ::ID &a) const;
    const SUMO::TAZ::ID                                   toSumoTAZ(const Environment::Node &a) const;

    const Environment::Edge::ID                           toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID                         toSumoEdge(const Environment::Edge::ID &a) const;
    const std::pair<Environment::Node, Environment::Node> toNodes(const SUMO::Network::Edge::ID &a) const;

    bool isSumoEdge(const Environment::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;

    SUMO::Network::Edge::ID fromNodeToSumoEdge(const Environment::Node &a) const;

    void clear();

    operator Static::SUMOAdapter &();

    void dump() const;
};
}  // namespace Dynamic
