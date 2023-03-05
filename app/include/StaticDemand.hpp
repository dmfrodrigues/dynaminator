#pragma once

#include "Graph.hpp"
#include "StaticNetwork.hpp"

#include <unordered_map>

class StaticDemand {
public:
    typedef StaticNetwork::Node Node;
    typedef StaticNetwork::Flow Flow;

private:
    std::unordered_map<Node, std::unordered_map<Node, Flow>> flows;

public:
    void addDemand(Node u, Node v, Flow f);
    Flow getDemand(Node u, Node v) const;
};
