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
    aon(aon_),
    solver(solver_),
    logger(logger_) {}

void ConjugateFrankWolfe::setStopCriteria(Cost e) {
    epsilon = e;
}

void ConjugateFrankWolfe::setIterations(int it) {
    iterations = it;
}

Solution ConjugateFrankWolfe::solve(
    const NetworkDifferentiable &network,
    const Demand                &dem,
    const Solution              &startingSolution
) {
    xStarStar = xn;

    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.

    supply = &network;
    demand = &dem;

    cout << fixed << setprecision(9);

    xn = startingSolution;
    zn = supply->evaluate(xn);

    double linearWithIterations = pow(-log10(epsilon / zn), 12);  // This variable has a linear relation with number of iterations
    double expectedIterations   = linearWithIterations / 272014.433647;

    double estimation1 = 0.176 * expectedIterations;

    const double ETA_DECAY = 0.1 / min(expectedIterations, (double)iterations);

    logger << Log::ProgressLogger::Elapsed(0)
           << Log::ProgressLogger::Progress(0)
           << Log::ProgressLogger::ETA(estimation1)
           << Log::ProgressLogger::StartText()
           << "it\talpha\tzn\tdelta\tlowerBound\tAbsGap\tRelGap\tt1\tt2"
           << Log::ProgressLogger::EndMessage();

    double t1 = 0, t2 = 0;

    Cost initialAbsoluteGap = 0;

    const hrc::time_point tStart = hrc::now();

    Flow znPrev = zn;
    for(int it = 0; it < iterations; ++it) {
        Cost delta       = znPrev - zn;
        Cost absoluteGap = zn - lowerBound;
        Cost relativeGap = absoluteGap / zn;

        const hrc::time_point t = hrc::now();

        double elapsed = (double)chrono::duration_cast<chrono::nanoseconds>(t - tStart).count() * 1e-9;

        // Progress
        if(it == 0) initialAbsoluteGap = absoluteGap;
        Cost progressEpsilon = pow(
            log(absoluteGap / initialAbsoluteGap) / log(epsilon / initialAbsoluteGap),
            12
        );
        Cost progressIterations = Cost(it + 1) / iterations;
        Cost progress           = max(progressEpsilon, progressIterations);
        progress                = max(0.0, min(1.0, progress));

        // ETA
        double estimation = estimation1;
        if(it > 0 && progress > 0) {
            double estimation2 = elapsed / progress;
            estimation1        = (1 - ETA_DECAY) * estimation1 + ETA_DECAY * estimation2;
            estimation         = (1 - progress * progress) * estimation1 + progress * progress * estimation2;
        }
        double eta = estimation - elapsed;

        progress = elapsed / estimation;

        logger << Log::ProgressLogger::Elapsed(elapsed)
               << Log::ProgressLogger::Progress(progress)
               << Log::ProgressLogger::ETA(eta)
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
            logger << Log::ProgressLogger::Elapsed(elapsed)
                   << Log::ProgressLogger::Progress(progress)
                   << Log::ProgressLogger::ETA(eta)
                   << Log::ProgressLogger::StartText()
                   << "Met relative gap criteria"
                   << Log::ProgressLogger::EndMessage();
            return xn;
        }

        znPrev = zn;

        hrc::time_point a = hrc::now();

        Solution xStar = step1();

        hrc::time_point b = hrc::now();

        xn = step2(xStar);

        hrc::time_point c = hrc::now();

        t1 = (double)chrono::duration_cast<chrono::nanoseconds>(b - a).count() * 1e-9;
        t2 = (double)chrono::duration_cast<chrono::nanoseconds>(c - b).count() * 1e-9;

        zn = supply->evaluate(xn);

        xStarStar = xStar;
    }

    return xn;
}

Solution ConjugateFrankWolfe::step1() {
    SolutionBase xAoN = aon.solve(*supply, *demand, xn);

    unordered_set<Edge::ID>        edgeIDs;
    const unordered_set<Edge::ID> &xnEdges    = xn.getEdges();
    const unordered_set<Edge::ID> &xStarEdges = xAoN.getEdges();
    edgeIDs.insert(xnEdges.begin(), xnEdges.end());
    edgeIDs.insert(xStarEdges.begin(), xStarEdges.end());

    // Conjugate
    double top = 0.0, bot = 0.0;
    for(const Edge::ID &eid: edgeIDs) {
        NetworkDifferentiable::Edge *e = supply->getEdge(eid);

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

    Solution xStar = Solution::interpolate(xAoN, xStarStar, a);

    // Update lower bound
    Cost zApprox = zn;
    for(const Edge::ID &eid: edgeIDs) {
        NetworkDifferentiable::Edge *e = supply->getEdge(eid);

        Flow xna    = xn.getFlowInEdge(eid);
        Flow xStara = xAoN.getFlowInEdge(eid);

        zApprox += e->calculateCost(xn) * (xStara - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    return xStar;
}

Solution ConjugateFrankWolfe::step2(const Solution &xstar) {
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
