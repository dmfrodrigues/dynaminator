#pragma once

#include <ctpl_stl.h>

#include "Log/ProgressLogger.hpp"
#include "Opt/UnivariateSolver.hpp"
#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/supply/Network.hpp"

namespace Static {
class FrankWolfe {
   protected:
    AllOrNothing          &aon;
    Opt::UnivariateSolver &solver;
    Log::ProgressLogger   &logger;

    const Network *supply;
    const Demand  *demand;

    Solution      xn;
    Network::Cost zn;
    Network::Cost epsilon;
    int           iterations = 1000;

    // Internal state
    Solution xStar;

    Opt::UnivariateSolver::Var alpha = 0.0;

    Network::Cost lowerBound = 0.0;

   public:
    FrankWolfe(
        AllOrNothing          &aon,
        Opt::UnivariateSolver &solver,
        Log::ProgressLogger   &logger
    );

    void setStopCriteria(Network::Cost e);
    void setIterations(int it);

    Solution solve(const Network &supply, const Demand &demand, const Solution &startingSolution);

   protected:
    virtual double getExpectedIterations();

    virtual Solution step1();
    virtual Solution step2(const Solution &xstar);
};
}  // namespace Static
