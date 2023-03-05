#include "StaticNetwork.hpp"

#include "StaticSolution.hpp"

#include "utils/strong_hash.hpp"

using namespace std;

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

size_t hash<StaticNetwork::Path>::operator()(const StaticNetwork::Path &v) const {
    return utils::strong_hash<vector<long>>()(v);
}
