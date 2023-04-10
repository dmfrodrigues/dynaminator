#pragma once

#include <ctpl_stl.h>

#include "opt/UnivariateSolver.hpp"
#include "static/StaticDemand.hpp"
#include "static/StaticSolution.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/StaticNetworkDifferentiable.hpp"

class ConjugateFrankWolfe {
    AllOrNothing &aon;
    UnivariateSolver &solver;

    const StaticNetworkDifferentiable *supply;
    const StaticDemand *demand;
    StaticSolution xn;
    StaticNetwork::Cost zn;
    StaticNetwork::Cost epsilon;
    int iterations = 1000;

    // Internal state
    StaticSolution xStarStar;
    UnivariateSolver::Var alpha = 0.0;
    StaticNetwork::Cost lowerBound = 0.0;

   public:
    ConjugateFrankWolfe(AllOrNothing &aon, UnivariateSolver &solver);

    void setStopCriteria(StaticNetwork::Cost e);
    void setIterations(int it);

    StaticSolution solve(
        const StaticNetworkDifferentiable &network,
        const StaticDemand &demand,
        const StaticSolution &startingSolution
    );

   private:
    StaticSolution step1();
    StaticSolution step2(const StaticSolution &xstar);
};
