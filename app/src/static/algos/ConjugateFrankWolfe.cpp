#include "static/algos/ConjugateFrankWolfe.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

#include "opt/QuadraticSolver.hpp"
#include "opt/UnivariateSolver.hpp"
#include "static/StaticSolution.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/algos/DijkstraAoN.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Edge Edge;
typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Cost Cost;

typedef chrono::high_resolution_clock hrc;

const double EPSILON = 1e-6;

ConjugateFrankWolfe::ConjugateFrankWolfe(
    AllOrNothing &aon_,
    UnivariateSolver &solver_
):
    aon(aon_),
    solver(solver_) {}

void ConjugateFrankWolfe::setStopCriteria(Cost e) {
    epsilon = e;
}

void ConjugateFrankWolfe::setIterations(int it) {
    iterations = it;
}

StaticSolution ConjugateFrankWolfe::solve(
    const StaticNetworkDifferentiable &network,
    const StaticDemand &dem,
    const StaticSolution &startingSolution
) {
    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.

    supply = &network;
    demand = &dem;

    xn = startingSolution;
    zn = supply->evaluate(xn);

    xStarStar = xn;

    cout
        << "FW algorithm\n"
        << "it\talpha\tzn\tdelta\tlowerBound\tAbsGap\tRelGap\tt1\tt2\n";
    cout << fixed << setprecision(9);

    double t1 = 0, t2 = 0;

    Flow znPrev = zn;
    for(int it = 0; it < iterations; ++it) {
        Cost delta = znPrev - zn;
        Cost absoluteGap = zn - lowerBound;
        Cost relativeGap = absoluteGap / zn;
        cout << it
             << "\t" << alpha
             << "\t" << zn
             << "\t" << delta
             << "\t" << lowerBound
             << "\t" << absoluteGap
             << "\t" << relativeGap
             << "\t" << t1
             << "\t" << t2
             << endl;

        znPrev = zn;

        hrc::time_point a = hrc::now();
        StaticSolution xStar = step1();
        hrc::time_point b = hrc::now();
        xn = step2(xStar);
        hrc::time_point c = hrc::now();

        t1 = (double)chrono::duration_cast<chrono::nanoseconds>(b - a).count() * 1e-9;
        t2 = (double)chrono::duration_cast<chrono::nanoseconds>(c - b).count() * 1e-9;

        zn = supply->evaluate(xn);

        if(absoluteGap <= epsilon) {
            cout << "FW: Met relative gap criteria. Stopping" << endl;
            return xn;
        }

        xStarStar = xStar;
    }

    return xn;
}

StaticSolution ConjugateFrankWolfe::step1() {
    StaticSolutionBase xAoN = aon.solve(*supply, *demand, xn);

    unordered_set<Edge::ID> edges;
    const unordered_set<Edge::ID> &xnEdges = xn.getEdges();
    const unordered_set<Edge::ID> &xStarEdges = xAoN.getEdges();
    edges.insert(xnEdges.begin(), xnEdges.end());
    edges.insert(xStarEdges.begin(), xStarEdges.end());

    // Conjugate
    double top = 0.0, bot = 0.0;
    for(const Edge::ID &eid: edges) {
        Flow xna = xn.getFlowInEdge(eid);
        Flow xAoNa = xAoN.getFlowInEdge(eid);
        Flow xStarStara = xStarStar.getFlowInEdge(eid);
        top += (xStarStara - xna) * (xAoNa - xna) * supply->calculateCostDerivative(eid, xna);
        bot += (xStarStara - xna) * (xAoNa - xStarStara) * supply->calculateCostDerivative(eid, xna);
    }
    
    double a;
    if(bot == 0.0) a = 0.0;
    else a = top/bot;
    
    a = max(0.0, min(1.0 - EPSILON, a));

    StaticSolution xStar = StaticSolution::interpolate(xAoN, xStarStar, a);

    // Update lower bound
    Cost zApprox = zn;
    for(const Edge::ID &eid: edges) {
        Flow xna = xn.getFlowInEdge(eid);
        Flow xStara = xAoN.getFlowInEdge(eid);
        zApprox += supply->calculateCost(eid, xna) * (xStara - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    return xStar;
}

StaticSolution ConjugateFrankWolfe::step2(const StaticSolution &xstar) {
    // TODO: allow to tune this value of epsilon
    UnivariateSolver::Problem p = [
        &supply = as_const(supply),
        &xn = as_const(xn),
        &xstar = as_const(xstar)
    ](UnivariateSolver::Var a) -> Cost {
        StaticSolution x = StaticSolution::interpolate(xn, xstar, a);
        StaticNetwork::Cost c = supply->evaluate(x);
        return c;
    };

    alpha = solver.solve(p);
    // if(alpha < 0.0) {
    //     cerr << "alpha (" << alpha << ") < 0, assuming alpha = 0" << endl;
    //     alpha = 0.0;
    // } else if(alpha > 1.0) {
    //     cerr << "alpha (" << alpha << ") > 1, assuming alpha = 1" << endl;
    //     alpha = 1.0;
    // }
    StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);

    return x;
}
