#pragma once

#include <list>

#include "Alg/Graph.hpp"

namespace Alg::ShortestPath {
/**
 * @brief Shortest Path From One Node to All other Nodes (Shortest Path One Many Interface)
 *
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
    virtual void solve(const Graph *G, const std::vector<Graph::Node> &s) = 0;

    /**
     * @brief Retrieves the node chosen prior to getting to node d
     *
     * @param d     Destination Node
     * @return Graph::Edge Edge traversed before getting to destination Node
     */
    virtual Graph::Edge getPrev(Graph::Node s, Graph::Node d) const = 0;

    // TODO: weigh in the advantages/disadvantages of getPath returning a vector, and evaluate performance if using a list.
    /**
     * @brief Retrieves the sequence of nodes of the path ending at d
     *
     * @param d Destination Node
     * @return std::list<Graph::Node> Sequence of nodes that describe the path to d
     */
    virtual Graph::Path getPath(Graph::Node s, Graph::Node d) const final;

    virtual Graph::Edge::Weight getPathWeight(Graph::Node s, Graph::Node d) const = 0;

    /**
     * @brief Checks if a specific node was marked as visited
     *
     * @param u         Node to be checked
     * @return true     If the node has been already visited
     * @return false    Otherwise
     */
    virtual bool hasVisited(Graph::Node s, Graph::Node u) const = 0;
};
}
