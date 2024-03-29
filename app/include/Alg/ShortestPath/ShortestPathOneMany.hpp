#pragma once

#include <list>

#include "Alg/Graph.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Shortest path from one node to all other nodes.
 */
class ShortestPathOneMany {
   private:
    virtual bool isStart(Graph::Node u) const = 0;

   public:
    virtual ~ShortestPathOneMany();

    /**
     * @brief Execute the algorithm
     *
     * @param G Graph
     * @param s Starting node
     */
    virtual void solve(const Graph &G, Graph::Node s) final;

    virtual void solveList(const Graph &G, const std::list<Graph::Node> &sList) = 0;

    /**
     * @brief Retrieves the edge that leads to d in the shortest path to d
     *
     * @param   d           Destination Node
     * @return  Graph::Edge Edge traversed before getting to d
     */
    virtual Graph::Edge getPrev(Graph::Node d) const = 0;

    /**
     * @brief Get shortest path (sequence of nodes) from starting node to d
     *
     * @param   d                       Destination node
     * @return  std::list<Graph::Node>  Shortest path to d
     */
    virtual Graph::Path getPath(Graph::Node d) const final;

    /**
     * @brief Get weight of shortest path from starting node to d
     *
     * @param   d                   Destination node
     * @return  Graph::Edge::Weight Weight of shortest path
     */
    virtual Graph::Edge::Weight getPathWeight(Graph::Node d) const = 0;

    /**
     * @brief Checks if a node was marked as visited
     *
     * @param u         Node to be checked
     * @return true     If the node has been already visited
     * @return false    Otherwise
     */
    virtual bool hasVisited(Graph::Node u) const = 0;
};
}  // namespace Alg::ShortestPath
