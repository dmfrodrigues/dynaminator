#pragma once

#include <ctpl_stl.h>

#include "opt/UnivariateSolver.hpp"
#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"

namespace Static {
class ConjugateFrankWolfe {
    AllOrNothing &aon;
    UnivariateSolver &solver;

    const NetworkDifferentiable *supply;
    const Demand *demand;
    Solution xn;
    Network::Cost zn;
    Network::Cost epsilon;
    int iterations = 1000;

    // Internal state
    Solution xStarStar;
    UnivariateSolver::Var alpha = 0.0;
    Network::Cost lowerBound = 0.0;

   public:
    ConjugateFrankWolfe(AllOrNothing &aon, UnivariateSolver &solver);

    void setStopCriteria(Network::Cost e);
    void setIterations(int it);

    Solution solve(
        const NetworkDifferentiable &network,
        const Demand &demand,
        const Solution &startingSolution
    );

   private:
    Solution step1();
    Solution step2(const Solution &xstar);
};
}
