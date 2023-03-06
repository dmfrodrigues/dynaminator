#pragma once

#include "StaticNetwork.hpp"

class BPRNetwork: public StaticNetwork {
    std::unordered_map<Node, std::vector<Edge*>> adj;

public:
    virtual std::vector<Node> getNodes() const;
    virtual const std::vector<Edge*> &getAdj(Node u) const;

    virtual Cost calculateCost(Edge::Id id, Flow f) const;

    virtual Cost evaluate(const StaticSolution &solution) const;
};
