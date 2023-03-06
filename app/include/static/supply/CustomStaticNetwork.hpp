#pragma once

#include <functional>

#include "static/supply/StaticNetwork.hpp"

class CustomStaticNetwork : public StaticNetwork {
   public:
    typedef std::function<double(double)> CostFunction;

   private:
    struct CustomEdge: public Edge {
        CostFunction cost;
    };

    std::unordered_map<Node, std::vector<CustomEdge*>> adj;
    std::unordered_map<Edge::Id, CustomEdge*> edges;

   public:
    void addNode(Node u);
    void addEdge(Edge::Id id, Node u, Node v, CostFunction);

    virtual std::vector<Node> getNodes() const;
    virtual std::vector<Edge *> getAdj(Node u) const;
    virtual Cost calculateCost(Edge::Id id, Flow f) const;

    ~CustomStaticNetwork();
};
