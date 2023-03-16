#pragma once

#include <unordered_map>
#include <utility>

#include "data/SumoNetwork.hpp"
#include "data/SumoTAZs.hpp"
#include "static/supply/StaticNetwork.hpp"

class SumoAdapterStatic {
    std::unordered_map<StaticNetwork::Node, SumoNetwork::Junction::Id> node2sumoJunction;
    std::unordered_map<SumoNetwork::Junction::Id, StaticNetwork::Node> sumoJunction2node;

    std::unordered_map<StaticNetwork::Node, SumoTAZs::TAZ::Id> node2sumoTAZ;
    std::unordered_map<SumoTAZs::TAZ::Id, std::pair<StaticNetwork::Node, StaticNetwork::Node>> sumoTAZ2node;

    std::unordered_map<StaticNetwork::Edge::Id, SumoNetwork::Edge::Id> edge2sumoEdge;
    std::unordered_map<SumoNetwork::Junction::Id, StaticNetwork::Node> sumoEdge2edge;

    StaticNetwork::Node nextNode = 0;
    StaticNetwork::Edge::Id nextEdge = 0;

   public:
    StaticNetwork::Node addSumoJunction(const SumoNetwork::Junction::Id &a);
    std::pair<StaticNetwork::Node, StaticNetwork::Node> addSumoTAZ(const SumoTAZs::TAZ::Id &a);
    StaticNetwork::Edge::Id addSumoEdge(const SumoNetwork::Edge::Id &a);

    const StaticNetwork::Node &toNode(const SumoNetwork::Junction::Id &a) const;
    const SumoNetwork::Junction::Id &toSumoJunction(const StaticNetwork::Node &a) const;

    const std::pair<StaticNetwork::Node, StaticNetwork::Node> &toTAZNode(const SumoTAZs::TAZ::Id &a) const;
    const SumoTAZs::TAZ::Id &toSumoTAZ(const StaticNetwork::Node &a) const;

    const StaticNetwork::Edge::Id &toEdge(const SumoNetwork::Edge::Id &a) const;
    const SumoNetwork::Edge::Id &toSumoEdge(const StaticNetwork::Edge::Id &a) const;
};
