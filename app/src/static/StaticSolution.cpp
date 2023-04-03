#include "static/StaticSolution.hpp"

#include <set>
#include <algorithm>

using namespace std;

typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Edge Edge;
typedef StaticNetwork::Path Path;


unordered_set<Edge::ID> StaticSolution::getEdges() const {
    return edges;
}

Flow StaticSolution::getFlowInEdge(Edge::ID id) const {
    if((Edge::ID)flows.size() <= id) return 0.0;
    else return flows[id];
}

/**
 * TODO: consider the possibility of not materializing an interpolated
 * StaticSolution; instead, implement new type that calculates flows on-the-fly
 * from the two original solutions being interpolated.
 */
StaticSolution StaticSolution::interpolate(
    const StaticSolution &s1,
    const StaticSolution &s2,
    Flow alpha
){
    StaticSolution ret;

    const unordered_set<Edge::ID> &edges1 = s1.getEdges();
    const unordered_set<Edge::ID> &edges2 = s2.getEdges();
    ret.edges.insert(edges1.begin(), edges1.end());
    ret.edges.insert(edges2.begin(), edges2.end());
    
    Edge::ID maxId = (ret.edges.empty() ? -1 : *max_element(ret.edges.begin(), ret.edges.end()));

    ret.flows.resize(maxId + 1, 0.0);
    for(size_t id = 0; id < ret.flows.size(); ++id){
        ret.flows[id] =
            (1.0-alpha) * s1.getFlowInEdge(id) +
            (    alpha) * s2.getFlowInEdge(id);
    }

    return ret;
}

void StaticSolutionBase::addPath(const Path &path, Flow newFlow){
    Flow &f = paths[path];
    Flow delta = newFlow-f;
    
    f = newFlow;

    Edge::ID maxId = (path.empty() ? -1 : *max_element(path.begin(), path.end()));
    if(maxId >= flows.size()) flows.resize(maxId+1, 0.0);

    for(const Edge::ID &id: path){
        edges.insert(id);
        flows[id] += delta;
    }
}
