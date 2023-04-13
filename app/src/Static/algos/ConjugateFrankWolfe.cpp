#include "Static/algos/ConjugateFrankWolfe.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

#include "Log/ProgressLogger.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Opt/UnivariateSolver.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/algos/FrankWolfe.hpp"

using namespace std;
using namespace Static;

typedef Network::Node Node;
typedef Network::Edge Edge;
typedef Network::Flow Flow;
typedef Network::Cost Cost;

typedef chrono::high_resolution_clock hrc;

const double EPSILON = 0.1;

ConjugateFrankWolfe::ConjugateFrankWolfe(
    AllOrNothing          &aon_,
    Opt::UnivariateSolver &solver_,
    Log::ProgressLogger   &logger_
):
    FrankWolfe(aon_, solver_, logger_) {}

Solution ConjugateFrankWolfe::solve(
    const NetworkDifferentiable &network,
    const Demand                &dem,
    const Solution              &startingSolution
) {
    supplyDifferentiable = &network;

    return FrankWolfe::solve(network, dem, startingSolution);
}

double ConjugateFrankWolfe::getExpectedIterations(){
    double linearWithIterations = pow(-log10(epsilon / zn), 12);  // This variable has a linear relation with number of iterations
    double expectedIterations   = linearWithIterations / 272014.433647;
    return expectedIterations;
}

Solution ConjugateFrankWolfe::step1() {
    SolutionBase xAoN = aon.solve(*supply, *demand, xn);

    unordered_set<Edge::ID>        edgeIDs;
    const unordered_set<Edge::ID> &xnEdges    = xn.getEdges();
    const unordered_set<Edge::ID> &xStarEdges = xAoN.getEdges();
    edgeIDs.insert(xnEdges.begin(), xnEdges.end());
    edgeIDs.insert(xStarEdges.begin(), xStarEdges.end());

    // Update lower bound
    Cost zApprox = zn;
    for(const Edge::ID &eid: edgeIDs) {
        Network::Edge *e = supply->getEdge(eid);

        Flow xna   = xn.getFlowInEdge(eid);
        Flow xAoNa = xAoN.getFlowInEdge(eid);

        zApprox += e->calculateCost(xn) * (xAoNa - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    // Get xStar

    // Store previous xStar value in xStarStar
    const Solution &xStarStar = xStar;

    double top = 0.0, bot = 0.0;
    for(const Edge::ID &eid: edgeIDs) {
        NetworkDifferentiable::Edge *e = supplyDifferentiable->getEdge(eid);

        Flow xna        = xn.getFlowInEdge(eid);
        Flow xAoNa      = xAoN.getFlowInEdge(eid);
        Flow xStarStara = xStarStar.getFlowInEdge(eid);

        top += (xStarStara - xna) * (xAoNa - xna) * e->calculateCostDerivative(xn);
        bot += (xStarStara - xna) * (xAoNa - xStarStara) * e->calculateCostDerivative(xn);
    }

    double a;
    if(bot == 0.0)
        a = 0.0;
    else
        a = top / bot;

    a = max(0.0, min(1.0 - EPSILON, a));

    xStar = Solution::interpolate(xAoN, xStarStar, a);

    return xStar;
}
