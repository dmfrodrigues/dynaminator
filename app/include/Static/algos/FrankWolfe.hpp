#pragma once

#include <ctpl_stl.h>

#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/supply/Network.hpp"
#include "opt/UnivariateSolver.hpp"

namespace Static {
class FrankWolfe {
    AllOrNothing     &aon;
    UnivariateSolver &solver;

    const Network *supply;
    const Demand  *demand;
    Solution       xn;
    Network::Cost  zn;
    Network::Cost  epsilon;
    int                  iterations = 1000;

    // Internal state
    UnivariateSolver::Var alpha      = 0.0;
    Network::Cost   lowerBound = 0.0;

   public:
    FrankWolfe(AllOrNothing &aon, UnivariateSolver &solver);

    void setStopCriteria(Network::Cost e);
    void setIterations(int it);

    Solution solve(const Network &supply, const Demand &demand, const Solution &startingSolution);

   private:
    SolutionBase step1();
    Solution     step2(const Solution &xstar);
};
}  // namespace Static
