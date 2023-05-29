#include "Static/supply/Network.hpp"

#include "Static/Solution.hpp"

#include "utils/strong_hash.hpp"

#include <iostream>

using namespace std;
using namespace Static;
using namespace Alg;

typedef Network::Edge Edge;
typedef Network::Time Time;

Network::Edge::Edge(ID id_, Node u_, Node v_):
    id(id_),
    u(u_),
    v(v_){}

Time Network::Edge::calculateDelay(const Solution &x) const {
    Time fft = calculateCost(SolutionBase());
    Time t = calculateCost(x);
    return (fft > 0.0 ? t/fft : 1.0);
}

Graph Network::toGraph(const Solution &solution) const {
    Graph G;
    
    const vector<Node> &nodes = getNodes();
    for(const Node &u: nodes)
        G.addNode(u);

    for(const Node &u: nodes){
        const vector<Edge*> &adj = getAdj(u);
        for(const Edge *e: adj){
            Time c = e->calculateCost(solution);
            G.addEdge(e->id, u, e->v, c);
        }
    }

    return G;
}

Time Network::evaluate(const Solution &solution) const {
    Time c = 0;

    unordered_set<Edge::ID> edgeIDs = solution.getEdges();
    for(const Edge::ID &eid: edgeIDs){
        Edge &e = getEdge(eid);
        c += e.calculateCostGlobal(solution);
    }

    return c;
}

size_t hash<Network::Path>::operator()(const Network::Path &v) const {
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
