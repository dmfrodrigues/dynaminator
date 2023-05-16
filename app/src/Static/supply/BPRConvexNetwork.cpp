#include "Static/supply/BPRConvexNetwork.hpp"

#include "Static/FixedSolution.hpp"

using namespace std;
using namespace Static;

typedef BPRConvexNetwork::Cost Cost;

BPRConvexNetwork::Edge::Edge(
    const BPRConvexNetwork &bprConvexNetwork_,
    const BPRNotConvexNetwork &bprNotConvex_,
    Edge::ID id_
):
    NetworkDifferentiable::Edge(
        id_,
        bprNotConvex_.getEdge(id_)->u,
        bprNotConvex_.getEdge(id_)->v
    ),
    bprConvex(bprConvexNetwork_),
    bprNotConvex(bprNotConvex_) {}

Cost BPRConvexNetwork::Edge::calculateCost(const Solution &x) const {
    FixedSolution x_ = bprConvex.solution;
    x_.setFlowInEdge(id, x.getFlowInEdge(id));
    return bprNotConvex.getEdge(id)->calculateCost(x);
}

Cost BPRConvexNetwork::Edge::calculateCostGlobal(const Solution &x) const {
    FixedSolution x_ = bprConvex.solution;
    x_.setFlowInEdge(id, x.getFlowInEdge(id));
    return bprNotConvex.getEdge(id)->calculateCostGlobal(x);
}

Cost BPRConvexNetwork::Edge::calculateCostDerivative(const Solution &x) const {
    FixedSolution x_ = bprConvex.solution;
    x_.setFlowInEdge(id, x.getFlowInEdge(id));
    return bprNotConvex.getEdge(id)->calculateCostDerivative(x);
}

BPRConvexNetwork::BPRConvexNetwork(const BPRNotConvexNetwork &bprNotConvex_, const Solution &solution_):
    bprNotConvex(bprNotConvex_), solution(solution_) {}

std::vector<Static::NetworkDifferentiable::Node> BPRConvexNetwork::getNodes() const {
    return bprNotConvex.getNodes();
}

BPRConvexNetwork::Edge *BPRConvexNetwork::getEdge(Edge::ID e) const {
    return new Edge(*this, bprNotConvex, e);
}

std::vector<Network::Edge *> BPRConvexNetwork::getAdj(Node u) const {
    std::vector<Network::Edge *> adj;
    for(auto e: bprNotConvex.getAdj(u)) {
        adj.push_back(new Edge(*this, bprNotConvex, e->id));
    }
    return adj;
}

void BPRConvexNetwork::saveResultsToFile(
    const SUMO::NetworkTAZs &sumo,
    const Solution &x,
    const SumoAdapterStatic &adapter,
    const std::string &edgeDataPath,
    const std::string &routesPath
) const {
    bprNotConvex.saveResultsToFile(sumo, x, adapter, edgeDataPath, routesPath);
}
