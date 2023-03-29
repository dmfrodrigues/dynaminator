#include "static/supply/StaticNetwork.hpp"

#include "static/StaticSolution.hpp"

#include "utils/strong_hash.hpp"

#include <iostream>

using namespace std;

typedef StaticNetwork::Edge Edge;
typedef StaticNetwork::Cost Cost;

Graph StaticNetwork::toGraph(const StaticSolution &solution) const {
    Graph G;
    
    const vector<Node> &nodes = getNodes();
    for(const Node &u: nodes)
        G.addNode(u);

    for(const Node &u: nodes){
        const vector<Edge*> &adj = getAdj(u);
        for(const Edge *e: adj){
            Cost c = calculateCost(e->id, solution.getFlowInEdge(e->id));
            G.addEdge(e->id, u, e->v, c);
        }
    }

    return G;
}

Cost StaticNetwork::evaluate(const StaticSolution &solution) const {
    Cost c = 0;

    vector<Edge::ID> edges = solution.getEdges();
    for(const Edge::ID &e: edges){
        Flow f = solution.getFlowInEdge(e);
        c += calculateCostGlobal(e, f);
    }

    return c;
}

size_t hash<StaticNetwork::Path>::operator()(const StaticNetwork::Path &v) const {
    // From:
    // - https://stackoverflow.com/a/72073933/12283316
    // This hash function only uses the size, first and last elements of vector.
    // It should be a somewhat bad but reasonable hash, that is lightning-fast
    // to calculate.
    size_t seed = v.size();
    seed ^= v.front() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= v.back() + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}
