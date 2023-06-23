#pragma once

#include <set>
#include <vector>

#include "ShortestPathOneMany.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Dijkstra's algorithm
 *
 */
class Dijkstra: public ShortestPathOneMany {
   private:
    std::vector<Graph::Edge::Weight> dist;
    std::vector<Graph::Edge>         prev;

    std::set<Graph::Node> sSet;

    virtual bool isStart(Graph::Node u) const;

   public:
    virtual void solveList(const Graph &G, const std::list<Graph::Node> &sList);

    virtual Graph::Edge getPrev(Graph::Node d) const;

    virtual Graph::Edge::Weight getPathWeight(Graph::Node d) const;

    virtual bool hasVisited(Graph::Node u) const;
};
}  // namespace Alg::ShortestPath
