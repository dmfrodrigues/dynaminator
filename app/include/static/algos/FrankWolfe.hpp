#pragma once

#include <ctpl_stl.h>

#include "convex/ConvexSolver.hpp"
#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class FrankWolfe {
    StaticProblem problem;

    StaticSolution xn;
    StaticNetwork::Cost zn;
    StaticNetwork::Cost epsilon;
    int iterations = 1000;

    ctpl::thread_pool pool = ctpl::thread_pool(8);

    // Internal state
    ConvexSolver::Var alpha = 0.0;
    StaticNetwork::Cost lowerBound = 0.0;

   public:
    FrankWolfe(StaticProblem prob);

    void setStartingSolution(const StaticSolution &startingSolution);
    void setStopCriteria(StaticNetwork::Cost e);
    void setIterations(int it);

    StaticSolution solve();

   private:
    StaticSolutionBase step1();
    StaticSolution step2(const StaticSolution &xstar);
};
