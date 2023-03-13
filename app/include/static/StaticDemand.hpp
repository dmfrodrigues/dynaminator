#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Graph.hpp"
#include "data/OFormatDemand.hpp"
#include "data/SumoNetwork.hpp"
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

    StaticNetwork::Flow getTotalDemand() const;

    static StaticDemand fromOFormat(
        const OFormatDemand &oDemand,
        const std::unordered_map<
            SumoNetwork::Junction::Id,
            std::pair<StaticNetwork::Node, StaticNetwork::Node> > &str2id_taz);
};
