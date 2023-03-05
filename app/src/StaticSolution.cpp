#include "StaticSolution.hpp"

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

StaticNetwork::Flow StaticSolution::getFlowInEdge(EdgeId id) const {
    const auto &it = flows.find(id);
    if(it == flows.end()) return 0.0;
    else return it->second;
}
