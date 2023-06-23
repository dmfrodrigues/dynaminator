#pragma once

#include <set>
#include <vector>

#include "Alg/ShortestPath/ShortestPathOneOne.hpp"
#include "ShortestPathOneMany.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Breadth-First Search algorithm
 *
 */
class BFS: public ShortestPathOneMany, public ShortestPathOneOne {
   private:
    typedef size_t Weight;

    static const Weight WEIGHT_INF;

    std::set<Graph::Node> sSet;

    std::vector<Weight>      dist;
    std::vector<Graph::Edge> prev;

    virtual bool isStart(Graph::Node u) const;

   public:
    virtual void                solveList(const Graph &G, const std::list<Graph::Node> &s);
    virtual Graph::Edge::Weight solveStartFinish(const Graph &G, Graph::Node s, Graph::Node t);

    Graph::Edge getPrev(Graph::Node d) const;

    Graph::Edge::Weight getPathWeight(Graph::Node d) const;

    bool hasVisited(Graph::Node u) const;
};
}  // namespace Alg::ShortestPath
