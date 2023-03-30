#pragma once

#include <unordered_map>
#include <utility>

#include "data/sumo/Network.hpp"
#include "data/sumo/TAZs.hpp"
#include "static/supply/StaticNetwork.hpp"

class SumoAdapterStatic {
    std::unordered_map<StaticNetwork::Edge::ID, SUMO::Network::Edge::ID> edge2sumoEdge;
    std::unordered_map<SUMO::Network::Edge::ID, StaticNetwork::Edge::ID> sumoEdge2edge;
    std::unordered_map<SUMO::Network::Edge::ID, std::pair<StaticNetwork::Node, StaticNetwork::Node>> sumoEdge2nodes;

    std::unordered_map<StaticNetwork::Node, SumoTAZs::TAZ::ID> node2sumoTAZ;
    std::unordered_map<SumoTAZs::TAZ::ID, std::pair<StaticNetwork::Node, StaticNetwork::Node>> sumoTAZ2node;

    StaticNetwork::Node nextNode = 0;
    StaticNetwork::Edge::ID nextEdge = 0;

   public:
    // StaticNetwork::Node addNode();
    std::pair<StaticNetwork::Node, StaticNetwork::Node> addSumoTAZ(const SumoTAZs::TAZ::ID &a);
    std::pair<StaticNetwork::Edge::ID, std::pair<StaticNetwork::Node, StaticNetwork::Node>> addSumoEdge(const SUMO::Network::Edge::ID &a);
    StaticNetwork::Edge::ID addEdge();
    bool isEdge(const SUMO::Network::Edge::ID &a) const;

    const std::pair<StaticNetwork::Node, StaticNetwork::Node> &toTAZNode(const SumoTAZs::TAZ::ID &a) const;
    const SumoTAZs::TAZ::ID &toSumoTAZ(const StaticNetwork::Node &a) const;

    const StaticNetwork::Edge::ID &toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID &toSumoEdge(const StaticNetwork::Edge::ID &a) const;
    const std::pair<StaticNetwork::Node, StaticNetwork::Node> &toNodes(const SUMO::Network::Edge::ID &a) const;

    std::vector<SUMO::Network::Edge::ID> getSumoEdges() const;
};
