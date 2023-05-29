#include "Static/algos/FrankWolfe.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

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
typedef Network::Time Cost;

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

    logger << std::fixed << std::setprecision(3);

    xn = startingSolution;
    zn = supply->evaluate(xn);

    xStar      = xn;
    alpha      = 0.0;
    lowerBound = 0;

    double expectedIterations = getExpectedIterations();

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

    Solution xnPrev = xn;
    Flow     znPrev = zn;
    for(size_t it = 0; it < iterations; ++it) {
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
        if(znPrev < zn) {
            logger << Log::ProgressLogger::Elapsed(elapsed)
                   << Log::ProgressLogger::Progress(progress)
                   << Log::ProgressLogger::ETA(eta)
                   << Log::ProgressLogger::StartText()
                   << "Solution got worse"
                   << Log::ProgressLogger::EndMessage();
            return xnPrev;
        }

        xnPrev = xn;
        znPrev = zn;

        hrc::time_point a = hrc::now();

        step1();

        hrc::time_point b = hrc::now();

        xn = step2(xStar);

        hrc::time_point c = hrc::now();

        t1 = (double)chrono::duration_cast<chrono::nanoseconds>(b - a).count() * 1e-9;
        t2 = (double)chrono::duration_cast<chrono::nanoseconds>(c - b).count() * 1e-9;

        zn = supply->evaluate(xn);
    }

    return xn;
}

double FrankWolfe::getExpectedIterations() {
    double linearWithIterations = pow(-log10(epsilon / zn), 12);  // This variable has a linear relation with number of iterations
    double expectedIterations   = linearWithIterations / 92259.0869806;
    return min(expectedIterations, (double)iterations);
}

Solution FrankWolfe::step1() {
    SolutionBase xAoN = aon.solve(*supply, *demand, xn);

    unordered_set<Edge::ID>        edgeIDs;
    const unordered_set<Edge::ID> &xnEdges   = xn.getEdges();
    const unordered_set<Edge::ID> &xAoNEdges = xAoN.getEdges();
    edgeIDs.insert(xnEdges.begin(), xnEdges.end());
    edgeIDs.insert(xAoNEdges.begin(), xAoNEdges.end());

    // Update lower bound
    Cost zApprox = zn;
    for(const Edge::ID &eid: edgeIDs) {
        Edge &e = supply->getEdge(eid);

        Flow xna   = xn.getFlowInEdge(eid);
        Flow xAoNa = xAoN.getFlowInEdge(eid);

        zApprox += e.calculateCost(xn) * (xAoNa - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    // Get xStar
    xStar = xAoN;

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
        Network::Time c = supply->evaluate(x);
        return c;
    };
    // clang-format on

    alpha = solver.solve(p);
    alpha = max(0.0, min(1.0, alpha));

    Solution x = Solution::interpolate(xn, xstar, alpha);

    return x;
}
