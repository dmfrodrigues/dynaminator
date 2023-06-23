#include "Alg/Flow/EdmondsKarp.hpp"

#include <unordered_set>

#include "Alg/Graph.hpp"

using namespace std;
using namespace Alg;
using namespace Alg::Flow;

typedef Graph::Node  Node;
typedef Graph::Edge  Edge;
typedef Edge::Weight Weight;
typedef Graph::Path  Path;

void EdmondsKarp::EGraph::addEdge(Edge::ID id, Node u, Node v, Edge::Weight c) {
    Graph::addEdge(id, u, v, c);
    if(edges.size() <= size_t(id))
        edges.resize(id + 1);
    edges.at(id) = {u, getAdj(u).size() - 1};
}

Edge &EdmondsKarp::EGraph::getEdge(Edge::ID id) {
    const auto &[u, i] = edges.at(id);
    return getAdj_(u).at(i);
}

EdmondsKarp::EdmondsKarp(ShortestPath::ShortestPathOneOne &sp_):
    sp(sp_) {}

void EdmondsKarp::buildResidualGraph(const Graph &G) {
    Gf = EGraph();

    for(const Node &u: G.getNodes()) {
        Gf.addNode(u);
        for(const Edge &e: G.getAdj(u)) {
            size_t eid1 = (size_t(e.id) << 1);
            size_t eid2 = (size_t(e.id) << 1) | 1;
            // clang-format off
            Gf.addEdge((Edge::ID)eid1, e.u, e.v, e.w);
            Gf.addEdge((Edge::ID)eid2, e.v, e.u, 0.0);
            // clang-format on
        }
    }
}

Weight getBottleneck(const Path &path) {
    Weight bottleneck = Edge::WEIGHT_INF;
    for(const Edge &e: path) {
        bottleneck = min(bottleneck, e.w);
    }
    return bottleneck;
}

Weight EdmondsKarp::solve(
    const Graph &G,
    Node         source,
    Node         sink
) {
    originalGraph = &G;

    buildResidualGraph(G);

    Weight totalFlow = 0.0;

    while(true) {
        sp.solveStartFinish(Gf, source, sink);
        if(!sp.hasVisited(sink))
            break;

        Path   path = sp.getPath(sink);
        Weight f    = getBottleneck(path);
        if(f <= 0.0)
            break;

        totalFlow += f;

        for(const Edge &e: path) {
            Edge::ID oppositeID = e.id ^ 1;

            Gf.getEdge(e.id).w -= f;
            Gf.getEdge(oppositeID).w += f;
        }
    }

    return totalFlow;
}

Graph EdmondsKarp::getFlowsGraph() const {
    unordered_set<Edge::ID> edgeIDs;
    for(const Node &u: originalGraph->getNodes()) {
        for(const Edge &e: originalGraph->getAdj(u)) {
            edgeIDs.insert(e.id);
        }
    }

    unordered_map<Edge::ID, Weight> flows;
    for(const Node &u: Gf.getNodes()) {
        for(const Edge &e: Gf.getAdj(u)) {
            Edge::ID eid      = e.id >> 1;
            bool     backward = (e.id & 1);
            if(backward && edgeIDs.count(eid)) {
                flows[e.id] = e.w;
            }
        }
    }

    Graph G;
    for(const Node &u: originalGraph->getNodes()) {
        G.addNode(u);
        for(const Edge &e: originalGraph->getAdj(u)) {
            G.addEdge(e.id, e.u, e.v, flows.at(e.id));
        }
    }

    return G;
}
