#pragma once

#include <ctpl_stl.h>

#include "ShortestPathManyMany.hpp"
#include "shortest-path/Dijkstra.hpp"

/**
 * @brief Dijkstra's algorithm for many start nodes
 *
 */
class DijkstraMany : public ShortestPathManyMany {
   private:
    std::unordered_map<Graph::Node, Dijkstra> dijkstras;
    ctpl::thread_pool pool;

   public:
    DijkstraMany(int parallelism = 8);

    void initialize(const Graph *G, const std::vector<Graph::Node> &s);

    void run();

    Graph::Edge getPrev(Graph::Node s, Graph::Node d) const;

    Graph::Edge::Weight getPathWeight(Graph::Node s, Graph::Node d) const;

    bool hasVisited(Graph::Node s, Graph::Node u) const;
};
