#pragma once

#include "Alg/ShortestPath/ShortestPathOneOne.hpp"
#include "ShortestPathOneMany.hpp"

#include <vector>

namespace Alg::ShortestPath {
/**
 * @brief Breadth-First Search algorithm
 * 
 */
class BFS : public ShortestPathOneMany, public ShortestPathOneOne {
private:
    typedef size_t Weight;

    static const Weight WEIGHT_INF;

    Graph::Node s;
    std::vector<Weight> dist;
    std::vector<Graph::Edge> prev;

    Graph::Node getStart() const;
public:

    void solve(const Graph &G, Graph::Node s);
    Graph::Edge::Weight solve(const Graph &G, Graph::Node s, Graph::Node t);

    Graph::Edge getPrev(Graph::Node d) const;

    Graph::Edge::Weight getPathWeight(Graph::Node d) const;

    bool hasVisited(Graph::Node u) const;
};
}
