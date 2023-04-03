#pragma once

#include <memory>
#include <unordered_set>

#include "supply/StaticNetwork.hpp"

class StaticSolutionBase;

class StaticSolution {
    friend StaticSolutionBase;

    struct Internals {
        std::unordered_map<StaticNetwork::Path, StaticNetwork::Flow> paths;
        std::unordered_set<StaticNetwork::Edge::ID> edges;
        std::vector<StaticNetwork::Flow> flows;
    };

    std::shared_ptr<Internals>
        s  = std::shared_ptr<Internals>(new Internals()),
        s1 = std::shared_ptr<Internals>(nullptr),
        s2 = std::shared_ptr<Internals>(nullptr);

   public:
    virtual std::unordered_set<StaticNetwork::Edge::ID> getEdges() const;
    virtual StaticNetwork::Flow getFlowInEdge(StaticNetwork::Edge::ID id) const;

    static StaticSolution interpolate(
        const StaticSolution &s1,
        const StaticSolution &s2,
        StaticNetwork::Flow alpha);
};

class StaticSolutionBase : public StaticSolution {
   public:
    void addPath(const StaticNetwork::Path &path, StaticNetwork::Flow flow);
};
