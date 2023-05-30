#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Alg/Graph.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/Network.hpp"
#include "Static/SUMOAdapter.hpp"
#include "data/VISUM/OFormatDemand.hpp"

namespace Static {
class Demand {
    std::unordered_map<
        Network::Node,
        std::unordered_map<
            Network::Node,
            Network::Flow> >
        flows;

   public:
    void addDemand(Network::Node u, Network::Node v, Network::Flow f);

    std::vector<Network::Node> getStartNodes() const;
    std::vector<Network::Node> getDestinations(Network::Node u) const;

    Network::Flow getDemand(Network::Node u, Network::Node v) const;
    Network::Flow getTotalDemand() const;

    static Demand fromOFormat(
        const VISUM::OFormatDemand &oDemand,
        const SUMOAdapter    &adapter
    );
};
}  // namespace Static
