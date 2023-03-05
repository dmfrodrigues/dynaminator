#pragma once

#include "StaticNetwork.hpp"

class StaticSolution {
    std::unordered_map<StaticNetwork::Path, StaticNetwork::Flow> paths;
    std::unordered_map<StaticNetwork::Edge::Id, StaticNetwork::Flow> flows;

public:
    void addPath(const StaticNetwork::Path &path, StaticNetwork::Flow f);

    StaticNetwork::Flow getFlowInEdge(StaticNetwork::Edge::Id id) const;
};
