#pragma once

#include "ShortestPathOneMany.hpp"

#include <vector>

/**
 * @brief Dijkstra's algorithm
 * 
 */
class Dijkstra : public ShortestPathOneMany {
private:
    const Graph *G;
    Graph::Node s;
    std::vector<Graph::Edge::Weight> dist;
    std::vector<Graph::Edge> prev;

    Graph::Node getStart() const;
public:

    void solve(const Graph *G, Graph::Node s);

    Graph::Edge getPrev(Graph::Node d) const;
    
    Graph::Edge::Weight getPathWeight(Graph::Node d) const;

    bool hasVisited(Graph::Node u) const;
};
