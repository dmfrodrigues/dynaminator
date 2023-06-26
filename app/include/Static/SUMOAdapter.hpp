#pragma once

#include <iostream>
#include <unordered_map>
#include <utility>

#include "Static/supply/Network.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"

namespace Static {
class SUMOAdapter {
    std::unordered_map<Network::Edge::ID, SUMO::Network::Edge::ID>                       edge2sumoEdge;
    std::unordered_map<SUMO::Network::Edge::ID, Network::Edge::ID>                       sumoEdge2edge;
    std::unordered_map<SUMO::Network::Edge::ID, std::pair<Network::Node, Network::Node>> sumoEdge2nodes;
    std::unordered_map<Network::Node, SUMO::Network::Edge::ID>                           node2sumoEdge;

    std::unordered_map<Network::Node, SUMO::TAZ::ID>                           node2sumoTAZ;
    std::unordered_map<SUMO::TAZ::ID, std::pair<Network::Node, Network::Node>> sumoTAZ2node;

    Network::Node     nextNode = 0;
    Network::Edge::ID nextEdge = 0;

   public:
    std::pair<Network::Node, Network::Node>                               addSumoTAZ(const SUMO::TAZ::ID &a);
    std::pair<Network::Edge::ID, std::pair<Network::Node, Network::Node>> addSumoEdge(const SUMO::Network::Edge::ID &a);
    Network::Edge::ID                                                     addEdge();
    bool                                                                  isEdge(const SUMO::Network::Edge::ID &a) const;

    const std::pair<Network::Node, Network::Node> &toTAZNode(const SUMO::TAZ::ID &a) const;
    const SUMO::TAZ::ID                           &toSumoTAZ(const Network::Node &a) const;

    const Network::Edge::ID                       &toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID                 &toSumoEdge(const Network::Edge::ID &a) const;
    const std::pair<Network::Node, Network::Node> &toNodes(const SUMO::Network::Edge::ID &a) const;

    bool isSumoEdge(const Network::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;

    SUMO::Network::Edge::ID fromNodeToSumoEdge(const Network::Node &a) const;

    void clear();

    void dump(std::ostream &os = std::cerr) const;
};
}  // namespace Static
