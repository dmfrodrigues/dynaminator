#pragma once

#include <ctpl_stl.h>

#include "opt/UnivariateSolver.hpp"
#include "static/StaticDemand.hpp"
#include "static/StaticSolution.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/StaticNetwork.hpp"

class FrankWolfe {
    AllOrNothing &aon;
    UnivariateSolver &solver;

    const StaticNetwork *supply;
    const StaticDemand *demand;
    StaticSolution xn;
    StaticNetwork::Cost zn;
    StaticNetwork::Cost epsilon;
    int iterations = 1000;

    // Internal state
    UnivariateSolver::Var alpha = 0.0;
    StaticNetwork::Cost lowerBound = 0.0;

   public:
    FrankWolfe(AllOrNothing &aon, UnivariateSolver &solver);

    void setStopCriteria(StaticNetwork::Cost e);
    void setIterations(int it);

    StaticSolution solve(const StaticNetwork &supply, const StaticDemand &demand, const StaticSolution &startingSolution);

   private:
    StaticSolutionBase step1();
    StaticSolution step2(const StaticSolution &xstar);
};
