#pragma once

#include <list>

#include "Alg/Graph.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Shortest path between a set of start nodes to all other nodes
 */
class ShortestPathManyMany {
   public:
    virtual ~ShortestPathManyMany();

    /**
     * @brief Execute the algorithm
     *
     * @param G Graph
     * @param s Starting nodes
     */
    virtual void solve(const Graph &G, const std::vector<Graph::Node> &s) = 0;

    /**
     * @brief Retrieves the node that comes before d in the path from s to d
     *
     * @param   s           Start node
     * @param   d           Destination node
     * @return  Graph::Edge Edge traversed before getting to d
     */
    virtual Graph::Edge getPrev(Graph::Node s, Graph::Node d) const = 0;

    /**
     * @brief Get shortest path (sequence of nodes) from s to d
     *
     * @param   s                       Start node
     * @param   d                       Destination node
     * @return  std::list<Graph::Node>  Shortest path to d
     */
    virtual Graph::Path getPath(Graph::Node s, Graph::Node d) const final;

    /**
     * @brief Get weight of shortest path from s to d
     *
     * @param   s                   Start node
     * @param   d                   Destination node
     * @return  Graph::Edge::Weight Weight of shortest path
     */
    virtual Graph::Edge::Weight getPathWeight(Graph::Node s, Graph::Node d) const = 0;

    /**
     * @brief Checks if a specific node was marked as visited when starting the
     * search at s
     *
     * @param   s                   Start node
     * @param u         Node to be checked
     * @return true     If the node has been already visited
     * @return false    Otherwise
     */
    virtual bool hasVisited(Graph::Node s, Graph::Node u) const = 0;
};
}  // namespace Alg::ShortestPath
