#pragma once

#include <ctpl_stl.h>

#include "opt/UnivariateSolver.hpp"
#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class FrankWolfe {
    const StaticProblem *problem;
    StaticSolution xn;
    StaticNetwork::Cost zn;
    StaticNetwork::Cost epsilon;
    int iterations = 1000;

    // Internal state
    UnivariateSolver::Var alpha = 0.0;
    StaticNetwork::Cost lowerBound = 0.0;

   public:
    void setStopCriteria(StaticNetwork::Cost e);
    void setIterations(int it);

    StaticSolution solve(const StaticProblem &prob, const StaticSolution &startingSolution);

   private:
    StaticSolutionBase step1();
    StaticSolution step2(const StaticSolution &xstar);
};
