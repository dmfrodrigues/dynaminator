#pragma once

#include "Alg/Flow/MaxFlow.hpp"
#include "Alg/Graph.hpp"
#include "Alg/ShortestPath/ShortestPathOneOne.hpp"

namespace Alg::Flow {
/**
 * @brief Maximum flow between two pairs of nodes.
 *
 * In these algorithms, the weight of edges is considered to be their capacities.
 */
class EdmondsKarp: public MaxFlow {
    ShortestPath::ShortestPathOneOne &sp;

    const Graph *originalGraph = nullptr;

    class EGraph: public Graph {
        std::vector<std::pair<Graph::Node, size_t>> edges;

       public:
        void addEdge(Edge::ID id, Node u, Node v, Edge::Weight c);
        Graph::Edge &getEdge(Graph::Edge::ID);
    };

    // Residual graph
    EGraph Gf;

    void buildResidualGraph(const Graph &G);

   public:
    EdmondsKarp(ShortestPath::ShortestPathOneOne &sp);

    virtual Graph::Edge::Weight solve(
        const Graph &G,
        Graph::Node  source,
        Graph::Node  sink
    );
    virtual Graph getFlowsGraph() const;
};
}  // namespace Alg::Flow
