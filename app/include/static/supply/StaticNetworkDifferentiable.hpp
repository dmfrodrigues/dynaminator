#pragma once

#include <unordered_map>
#include <vector>
#include <string>

#include "Graph.hpp"
#include "static/supply/StaticNetwork.hpp"

class StaticSolution;

class SumoAdapterStatic;

class StaticNetworkDifferentiable: public StaticNetwork {
    virtual Cost calculateCostDerivative(Edge::ID id, Flow f) const = 0;
};
