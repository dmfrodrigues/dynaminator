#pragma once

#include <ctpl_stl.h>

#include "Log/ProgressLogger.hpp"
#include "Opt/UnivariateSolver.hpp"
#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/supply/NetworkDifferentiable.hpp"

namespace Static {
class ConjugateFrankWolfe: private FrankWolfe {
    const NetworkDifferentiable *supplyDifferentiable;

   public:
    ConjugateFrankWolfe(
        AllOrNothing          &aon,
        Opt::UnivariateSolver &solver,
        Log::ProgressLogger   &logger
    );

    using FrankWolfe::setIterations;
    using FrankWolfe::setStopCriteria;

    Solution solve(
        const NetworkDifferentiable &network,
        const Demand                &demand,
        const Solution              &startingSolution
    );

   protected:
    virtual double getExpectedIterations();

    virtual Solution step1();
};
}  // namespace Static
