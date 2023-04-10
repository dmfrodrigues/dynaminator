#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Graph.hpp"
#include "Static/supply/Network.hpp"

class SumoAdapterStatic;

namespace Static {
class Solution;

class NetworkDifferentiable: public Network {
   public:
    virtual Cost calculateCostDerivative(Edge::ID id, Flow f) const = 0;
};

}  // namespace Static
