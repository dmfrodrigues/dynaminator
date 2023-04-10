#include "static/StaticSolution.hpp"

#include <set>
#include <algorithm>
#include <unordered_map>

using namespace std;

typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Edge Edge;
typedef StaticNetwork::Path Path;

void StaticSolution::Internals::addToRoutes(
    unordered_map<Path, Flow> &routes
) const {
    for(const auto &p: paths){
        const Path &path = p.first;
        const Flow &flow = p.second;
        routes[path] += flow;
    }
    if(s1 != nullptr) s1->addToRoutes(routes);
    if(s2 != nullptr) s2->addToRoutes(routes);
}

unordered_set<Edge::ID> StaticSolution::getEdges() const {
    const auto &edges = s.get()->edges;
    return edges;
}

Flow StaticSolution::getFlowInEdge(Edge::ID id) const {
    const auto &flows = s.get()->flows;
    if((Edge::ID)flows.size() <= id) return 0.0;
    else return flows[id];
}

unordered_map<Path, Flow> StaticSolution::getRoutes() const {
    unordered_map<Path, Flow> ret;
    s->addToRoutes(ret);
    return ret;
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

    ret.s->s1 = s1.s;
    ret.s->s2 = s2.s;

    const unordered_set<Edge::ID> &edges1 = s1.getEdges();
    const unordered_set<Edge::ID> &edges2 = s2.getEdges();

    auto &edges = ret.s.get()->edges;
    auto &flows = ret.s.get()->flows;

    edges.insert(edges1.begin(), edges1.end());
    edges.insert(edges2.begin(), edges2.end());
    
    Edge::ID maxId = (edges.empty() ?
        -1 :
        *max_element(edges.begin(), edges.end()));

    flows.resize(maxId + 1, 0.0);
    for(size_t id = 0; id < flows.size(); ++id){
        flows[id] =
            (1.0-alpha) * s1.getFlowInEdge(id) +
            (    alpha) * s2.getFlowInEdge(id);
    }

    return ret;
}

void StaticSolutionBase::addPath(const Path &path, Flow newFlow){
    auto &paths = s.get()->paths;
    auto &flows = s.get()->flows;
    auto &edges = s.get()->edges;

    Flow &f = paths[path];
    Flow delta = newFlow-f;
    
    f = newFlow;

    Edge::ID maxId = (path.empty() ? -1 : *max_element(path.begin(), path.end()));
    if(maxId >= (Edge::ID)flows.size()) flows.resize(maxId+1, 0.0);

    for(const Edge::ID &id: path){
        edges.insert(id);
        flows[id] += delta;
    }
}
