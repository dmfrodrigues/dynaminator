#pragma once

#include <memory>
#include <unordered_set>

#include "Static/Solution.hpp"
#include "Static/supply/Network.hpp"

namespace Static {
class FixedSolution: public Solution {
    std::unordered_map<Network::Edge::ID, Flow> flows;

   public:
    FixedSolution(const Solution &sol);

    void setFlowInEdge(Network::Edge::ID id, Flow flow);

    Flow getFlowInEdge(Network::Edge::ID id) const;
};
}  // namespace Static
