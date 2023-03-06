#include "StaticSolution.hpp"

using namespace std;

typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Edge::Id EdgeId;
typedef StaticNetwork::Path Path;

void StaticSolution::addPath(const Path &path, Flow newFlow){
    Flow &f = paths[path];
    Flow delta = newFlow-f;
    f = newFlow;
    for(const EdgeId &id: path){
        flows[id] += delta;
    }
}

vector<EdgeId> StaticSolution::getEdges() const {
    vector<EdgeId> ret;
    ret.reserve(flows.size());
    for(const auto &p: flows)
        ret.push_back(p.first);
    return ret;
}

StaticNetwork::Flow StaticSolution::getFlowInEdge(EdgeId id) const {
    const auto &it = flows.find(id);
    if(it == flows.end()) return 0.0;
    else return it->second;
}

/**
 * TODO: consider the possibility of not materializing an interpolated
 * StaticSolution; instead, implement new type that calculates flows on-the-fly
 * from the two original solutions being interpolated.
 */
StaticSolution StaticSolution::interpolate(
    const StaticSolution &s1,
    const StaticSolution &s2,
    double alpha
){
    StaticSolution ret;

    ret.paths.insert(s1.paths.begin(), s1.paths.end());
    ret.paths.insert(s2.paths.begin(), s2.paths.end());

    for(const auto &p: s1.flows){
        const EdgeId &e = p.first;
        ret.flows[e] += p.second;
    }
    for(const auto &p: s2.flows){
        const EdgeId &e = p.first;
        ret.flows[e] += p.second;
    }

    return ret;
}
