#pragma once

#include <utility>

#include "shortest-path/ShortestPathAll.hpp"

class DijkstraCuda : public ShortestPathAll {
    std::vector<Graph::Edge> edges;
    std::vector<std::pair<uint32_t, uint32_t>> adj;

    size_t numberStartNodes;
    Graph::Node *startNodes;
    
    Graph::Edge **prev;
    Graph::Edge::Weight **dist;

   public:
    virtual void initialize(const Graph *G, const std::list<Graph::Node> &s);
    virtual void run();
    virtual Graph::Edge getPrev(Graph::Node s, Graph::Node d) const;
    virtual Graph::Edge::Weight getPathWeight(Graph::Node s, Graph::Node d) const;
    virtual bool hasVisited(Graph::Node s, Graph::Node u) const;

    ~DijkstraCuda();
};
