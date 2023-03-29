#pragma once

#include "supply/StaticNetwork.hpp"

class StaticSolution {
    std::unordered_map<StaticNetwork::Path, StaticNetwork::Flow> paths;
    std::unordered_map<StaticNetwork::Edge::ID, StaticNetwork::Flow> flows;

public:
    void addPath(const StaticNetwork::Path &path, StaticNetwork::Flow f);

    std::vector<StaticNetwork::Edge::ID> getEdges() const;

    StaticNetwork::Flow getFlowInEdge(StaticNetwork::Edge::ID id) const;

    static StaticSolution interpolate(
        const StaticSolution &s1,
        const StaticSolution &s2,
        StaticNetwork::Flow alpha
    );
};
