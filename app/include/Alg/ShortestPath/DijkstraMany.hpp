#pragma once

#include <ctpl_stl.h>

#include "ShortestPathManyMany.hpp"
#include "Alg/ShortestPath/Dijkstra.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Dijkstra's algorithm for many start nodes
 *
 */
class DijkstraMany : public ShortestPathManyMany {
   private:
    std::unordered_map<Graph::Node, Dijkstra> dijkstras;
    ctpl::thread_pool pool;

   public:
    /**
     * @brief Construct a new DijkstraMany object
     *
     * @param parallelism Degree of parallelism when running many instances of
     * Dijkstra's algorithm simultaneously. If 0, all instances of Dijkstra's
     * algorithm are run sequentially in the current thread. If greater than 0,
     * a pool of size parallelism is created, and each instance of Dijkstra's
     * algorithm is executed in a separate task submitted to the pool.
     */
    DijkstraMany(int parallelism = 8);

    void solve(const Graph &G, const std::vector<Graph::Node> &s);

    Graph::Edge getPrev(Graph::Node s, Graph::Node d) const;

    Graph::Edge::Weight getPathWeight(Graph::Node s, Graph::Node d) const;

    bool hasVisited(Graph::Node s, Graph::Node u) const;
};
}
