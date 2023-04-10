#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Graph.hpp"
#include "static/supply/StaticNetwork.hpp"

class StaticSolution;

class SumoAdapterStatic;

class StaticNetworkDifferentiable: public StaticNetwork {
   public:
    virtual Cost calculateCostDerivative(Edge::ID id, Flow f) const = 0;
};
