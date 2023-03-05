#pragma once

#include "Graph.hpp"

/**
 * @brief Shortest Path From One Node to All other Nodes (Shortest Path One Many Interface)
 * 
 */
class ShortestPathOneMany {
private:

    virtual Graph::Node getStart() const = 0;
public:

    virtual ~ShortestPathOneMany();

    /**
     * @brief Initializes the data members that are required for the algorithm's execution
     * 
     * @param G Graph
     * @param s Starting node
     */
    virtual void initialize(const Graph *G, Graph::Node s) = 0;

    /**
     * @brief Execute the algorithm
     * 
     */
    virtual void run() = 0;

    /**
     * @brief Retrieves the node chosen prior to getting to node d
     * 
     * @param d     Destination Node
     * @return Graph::Node Last Node before getting to the destination Node
     */
    virtual Graph::Node getPrev(Graph::Node d) const = 0;

    /**
     * @brief Retrieves the sequence of nodes of the path ending at d
     * 
     * @param d Destination Node
     * @return std::list<Graph::Node> Sequence of nodes that describe the path to d
     */
    virtual std::vector<Graph::Node> getPath(Graph::Node d) const final;

    virtual Graph::Edge::Weight getPathWeight(Graph::Node d) const = 0;

    /**
     * @brief Checks if a specific node was marked as visited
     * 
     * @param u         Node to be checked
     * @return true     If the node has been already visited
     * @return false    Otherwise
     */
    virtual bool hasVisited(Graph::Node u) const = 0;
};
