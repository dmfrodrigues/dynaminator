#pragma once

#include "static/StaticDemand.hpp"
#include "static/StaticSolution.hpp"
#include "static/supply/StaticNetwork.hpp"

class AllOrNothing {
   public:
    virtual StaticSolutionBase solve(
        const StaticNetwork &supply,
        const StaticDemand &demand,
        const StaticSolution &flows = StaticSolutionBase()
    ) = 0;
};
