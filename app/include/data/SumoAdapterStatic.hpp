#pragma once

#include <unordered_map>
#include <utility>

#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"
#include "Static/supply/Network.hpp"

class SumoAdapterStatic {
    std::unordered_map<Static::Network::Edge::ID, SUMO::Network::Edge::ID> edge2sumoEdge;
    std::unordered_map<SUMO::Network::Edge::ID, Static::Network::Edge::ID> sumoEdge2edge;
    std::unordered_map<SUMO::Network::Edge::ID, std::pair<Static::Network::Node, Static::Network::Node>> sumoEdge2nodes;

    std::unordered_map<Static::Network::Node, SUMO::TAZ::ID> node2sumoTAZ;
    std::unordered_map<SUMO::TAZ::ID, std::pair<Static::Network::Node, Static::Network::Node>> sumoTAZ2node;

    Static::Network::Node nextNode = 0;
    Static::Network::Edge::ID nextEdge = 0;

   public:
    // Static::Network::Node addNode();
    std::pair<Static::Network::Node, Static::Network::Node> addSumoTAZ(const SUMO::TAZ::ID &a);
    std::pair<Static::Network::Edge::ID, std::pair<Static::Network::Node, Static::Network::Node>> addSumoEdge(const SUMO::Network::Edge::ID &a);
    Static::Network::Edge::ID addEdge();
    bool isEdge(const SUMO::Network::Edge::ID &a) const;

    const std::pair<Static::Network::Node, Static::Network::Node> &toTAZNode(const SUMO::TAZ::ID &a) const;
    const SUMO::TAZ::ID &toSumoTAZ(const Static::Network::Node &a) const;

    const Static::Network::Edge::ID &toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID &toSumoEdge(const Static::Network::Edge::ID &a) const;
    const std::pair<Static::Network::Node, Static::Network::Node> &toNodes(const SUMO::Network::Edge::ID &a) const;

    bool isSumoEdge(const Static::Network::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;

    void clear();
};
