#include "StaticDemand.hpp"

void StaticDemand::addDemand(Node u, Node v, Flow f){
    flows[u][v] += f;
}

StaticDemand::Flow StaticDemand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}
