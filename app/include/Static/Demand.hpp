#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Alg/Graph.hpp"
#include "Static/SUMOAdapter.hpp"
#include "Static/Static.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/Network.hpp"
#include "data/VISUM/OFormatDemand.hpp"

namespace Static {
class Demand {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Demand load(T arg1, args... arg2);
    };

   private:
    // clang-format off
    std::unordered_map<
        Network::Node,
        std::unordered_map<
            Network::Node,
            Flow
        >
    > flows;
    // clang-format on

   public:
    void addDemand(Network::Node u, Network::Node v, Flow f);

    std::vector<Network::Node> getStartNodes() const;
    std::vector<Network::Node> getDestinations(Network::Node u) const;

    Flow getDemand(Network::Node u, Network::Node v) const;
    Flow getTotalDemand() const;
};

template<>
class Demand::Loader<
    const VISUM::OFormatDemand &,
    const Static::SUMOAdapter &> {
   public:
    Demand load(
        const VISUM::OFormatDemand &oDemand,
        const SUMOAdapter          &adapter
    );
};

}  // namespace Static
