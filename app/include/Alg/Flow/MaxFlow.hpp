#pragma once

#include "Alg/Graph.hpp"

namespace Alg::Flow {
/**
 * @brief Maximum flow between two pairs of nodes.
 *
 * In these algorithms, the weight of edges is considered to be their capacities.
 */
class MaxFlow {
public:

    virtual ~MaxFlow();

    /**
     * @brief Execute the algorithm
     *
     * @param G Graph
     * @param source Source node
     * @param sink Sink node
     * @return Graph::Edge::Weight Maximum flow between source and sink
     */
    virtual Graph::Edge::Weight solve(const Graph &G, Graph::Node source, Graph::Node sink) = 0;

    /**
     * @brief Retrieves the flows graph.
     *
     * The flows graph is a copy of the original graph, but where edges' weights
     * represent the flow that traverses them.
     *
     * @return  Graph::Edge Edge traversed before getting to d
     */
    virtual Graph getFlowsGraph() const = 0;
};
}
