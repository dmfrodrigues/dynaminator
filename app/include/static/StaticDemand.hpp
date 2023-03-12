#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Graph.hpp"
#include "supply/StaticNetwork.hpp"

class StaticDemand {
    std::unordered_map<
        StaticNetwork::Node,
        std::unordered_map<
            StaticNetwork::Node,
            StaticNetwork::Flow> >
        flows;

   public:
    void addDemand(StaticNetwork::Node u, StaticNetwork::Node v, StaticNetwork::Flow f);
    std::vector<StaticNetwork::Node> getStartNodes() const;
    std::vector<StaticNetwork::Node> getDestinations(StaticNetwork::Node u) const;
    StaticNetwork::Flow getDemand(StaticNetwork::Node u, StaticNetwork::Node v) const;
};
