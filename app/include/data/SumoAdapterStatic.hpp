#pragma once

#include <unordered_map>
#include <utility>

#include "data/sumo/Network.hpp"
#include "data/sumo/TAZs.hpp"
#include "static/supply/StaticNetwork.hpp"

class SumoAdapterStatic {
    std::unordered_map<StaticNetwork::Node, SUMO::Network::Junction::ID> node2sumoJunction;
    std::unordered_map<SUMO::Network::Junction::ID, StaticNetwork::Node> sumoJunction2node;

    std::unordered_map<StaticNetwork::Node, SumoTAZs::TAZ::ID> node2sumoTAZ;
    std::unordered_map<SumoTAZs::TAZ::ID, std::pair<StaticNetwork::Node, StaticNetwork::Node>> sumoTAZ2node;

    std::unordered_map<StaticNetwork::Edge::ID, SUMO::Network::Edge::ID> edge2sumoEdge;
    std::unordered_map<SUMO::Network::Junction::ID, StaticNetwork::Node> sumoEdge2edge;

    StaticNetwork::Node nextNode = 0;
    StaticNetwork::Edge::ID nextEdge = 0;

   public:
    StaticNetwork::Node addSumoJunction(const SUMO::Network::Junction::ID &a);
    std::pair<StaticNetwork::Node, StaticNetwork::Node> addSumoTAZ(const SumoTAZs::TAZ::ID &a);
    StaticNetwork::Edge::ID addSumoEdge(const SUMO::Network::Edge::ID &a);
    StaticNetwork::Edge::ID addSumoEdge();

    const StaticNetwork::Node &toNode(const SUMO::Network::Junction::ID &a) const;
    const SUMO::Network::Junction::ID &toSumoJunction(const StaticNetwork::Node &a) const;

    const std::pair<StaticNetwork::Node, StaticNetwork::Node> &toTAZNode(const SumoTAZs::TAZ::ID &a) const;
    const SumoTAZs::TAZ::ID &toSumoTAZ(const StaticNetwork::Node &a) const;

    const StaticNetwork::Edge::ID &toEdge(const SUMO::Network::Edge::ID &a) const;
    const SUMO::Network::Edge::ID &toSumoEdge(const StaticNetwork::Edge::ID &a) const;
};
