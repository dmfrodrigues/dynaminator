#include "Static/FixedSolution.hpp"
#include "Static/supply/Network.hpp"

using namespace std;
using namespace Static;

FixedSolution::FixedSolution(const Solution &sol): Solution(sol){}

void FixedSolution::setFlowInEdge(Network::Edge::ID id, Flow flow){
    flows[id] = flow;
}

Flow FixedSolution::getFlowInEdge(Network::Edge::ID id) const{
    const auto &it = flows.find(id);
    if(it != flows.end()) return it->second;
    else return Solution::getFlowInEdge(id);
}
