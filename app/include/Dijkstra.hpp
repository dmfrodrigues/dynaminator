#pragma once

#include "ShortestPathOneMany.hpp"

#include <unordered_map>

/**
 * @brief Dijkstra's algorithm
 * 
 */
class Dijkstra : public ShortestPathOneMany {
private:
    const Graph *G;
    Graph::Node s;
    std::unordered_map<Graph::Node, Graph::Edge::Weight> dist;
    std::unordered_map<Graph::Node, Graph::Node> prev;

    Graph::Node getStart() const;
public:
    /**
     * @brief Initializes the data members that are required for the algorithm's execution
     * 
     * @param G Directed Weighted Graph
     * @param s Starting Node
     */
    void initialize(const Graph *G, Graph::Node s);

    /**
     * @brief Execute the algorithm
     * 
     */
    void run();

    /**
     * @brief Retrieves the node chosen prior to getting to node d
     * 
     * @param d                 Destination Node
     * @return Graph::Node  Last Node before getting to the destination Node
     */
    Graph::Node getPrev(Graph::Node d) const;
    Graph::Edge::Weight getPathWeight(Graph::Node d) const;

    /**
     * @brief Checks if a specific node was marked as visited
     * 
     * @param u         Node to be checked
     * @return true     If the node has been already visited
     * @return false    Otherwise
     */
    bool hasVisited(Graph::Node u) const;
};
