#include "Static/algos/FrankWolfe.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

#include "Opt/QuadraticSolver.hpp"
#include "Opt/UnivariateSolver.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/algos/DijkstraAoN.hpp"

using namespace std;
using namespace Static;

typedef Network::Node Node;
typedef Network::Edge Edge;
typedef Network::Flow Flow;
typedef Network::Cost Cost;

typedef chrono::high_resolution_clock hrc;

FrankWolfe::FrankWolfe(
    AllOrNothing          &aon_,
    Opt::UnivariateSolver &solver_,
    Log::ProgressLogger   &logger_
):
    aon(aon_),
    solver(solver_),
    logger(logger_) {}

void FrankWolfe::setStopCriteria(Cost e) {
    epsilon = e;
}

void FrankWolfe::setIterations(int it) {
    iterations = it;
}

Solution FrankWolfe::solve(
    const Network  &network,
    const Demand   &dem,
    const Solution &startingSolution
) {
    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.

    supply = &network;
    demand = &dem;

    xn = startingSolution;
    zn = supply->evaluate(xn);

    cout << fixed << setprecision(9);

    double linearWithIterations = pow(-log10(epsilon / zn), 12);  // This variable has a linear relation with number of iterations
    double expectedIterations   = linearWithIterations / 81762.2159768;
    double eta                  = 0.176 * expectedIterations;

    logger << Log::ProgressLogger::ETA(eta);

    logger << Log::ProgressLogger::Progress(0)
           << Log::ProgressLogger::StartText()
           << "it\talpha\tzn\tdelta\tlowerBound\tAbsGap\tRelGap\tt1\tt2"
           << Log::ProgressLogger::EndMessage();

    double t1 = 0, t2 = 0;

    Cost initialAbsoluteGap = 0;

    Flow znPrev = zn;
    for(int it = 0; it < iterations; ++it) {
        Cost delta       = znPrev - zn;
        Cost absoluteGap = zn - lowerBound;
        Cost relativeGap = absoluteGap / zn;

        // Progress
        if(it == 0) initialAbsoluteGap = absoluteGap;
        Cost progressEpsilon = pow(
            log(absoluteGap / initialAbsoluteGap) / log(epsilon / initialAbsoluteGap),
            12
        );
        Cost progressIterations = Cost(it + 1) / iterations;
        Cost progress           = max(progressEpsilon, progressIterations);
        progress                = max(0.0, min(1.0, progress));

        logger << Log::ProgressLogger::Progress(progress)
               << Log::ProgressLogger::StartText()
               << it
               << "\t" << alpha
               << "\t" << zn
               << "\t" << delta
               << "\t" << lowerBound
               << "\t" << absoluteGap
               << "\t" << relativeGap
               << "\t" << t1
               << "\t" << t2
               << Log::ProgressLogger::EndMessage();

        if(absoluteGap <= epsilon) {
            cout << "FW: Met relative gap criteria. Stopping" << endl;
            return xn;
        }

        znPrev = zn;

        hrc::time_point a = hrc::now();

        SolutionBase xStar = step1();

        hrc::time_point b = hrc::now();

        xn = step2(xStar);

        hrc::time_point c = hrc::now();

        t1 = (double)chrono::duration_cast<chrono::nanoseconds>(b - a).count() * 1e-9;
        t2 = (double)chrono::duration_cast<chrono::nanoseconds>(c - b).count() * 1e-9;

        zn = supply->evaluate(xn);
    }

    return xn;
}

SolutionBase FrankWolfe::step1() {
    SolutionBase xStar = aon.solve(*supply, *demand, xn);

    // Update lower bound
    Cost                           zApprox = zn;
    unordered_set<Edge::ID>        edgeIDs;
    const unordered_set<Edge::ID> &xnEdges    = xn.getEdges();
    const unordered_set<Edge::ID> &xStarEdges = xStar.getEdges();
    edgeIDs.insert(xnEdges.begin(), xnEdges.end());
    edgeIDs.insert(xStarEdges.begin(), xStarEdges.end());
    for(const Edge::ID &eid: edgeIDs) {
        Edge *e      = supply->getEdge(eid);
        Flow  xna    = xn.getFlowInEdge(eid);
        Flow  xStara = xStar.getFlowInEdge(eid);
        zApprox += e->calculateCost(xn) * (xStara - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    return xStar;
}

Solution FrankWolfe::step2(const Solution &xstar) {
    // TODO: allow to tune this value of epsilon
    // clang-format off
    Opt::UnivariateSolver::Problem p = [
        &supply = as_const(supply),
        &xn     = as_const(xn),
        &xstar  = as_const(xstar)
    ](Opt::UnivariateSolver::Var a) -> Cost {
        Solution      x = Solution::interpolate(xn, xstar, a);
        Network::Cost c = supply->evaluate(x);
        return c;
    };
    // clang-format on

    alpha = solver.solve(p);
    // if(alpha < 0.0) {
    //     cerr << "alpha (" << alpha << ") < 0, assuming alpha = 0" << endl;
    //     alpha = 0.0;
    // } else if(alpha > 1.0) {
    //     cerr << "alpha (" << alpha << ") > 1, assuming alpha = 1" << endl;
    //     alpha = 1.0;
    // }
    Solution x = Solution::interpolate(xn, xstar, alpha);

    return x;
}
