#pragma once

#include <memory>
#include <unordered_set>

#include "Static/supply/Network.hpp"

#include "Static/Solution.hpp"

namespace Static {
class FixedSolution : public Solution {
    std::unordered_map<Network::Edge::ID, Network::Flow> flows;

   public:
    FixedSolution(const Solution &sol);

    void setFlowInEdge(Network::Edge::ID id, Network::Flow flow);

    Network::Flow getFlowInEdge(Network::Edge::ID id) const;
};
}  // namespace Static
