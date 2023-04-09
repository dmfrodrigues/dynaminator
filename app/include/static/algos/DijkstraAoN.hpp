#pragma once

#include "static/StaticDemand.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/supply/StaticNetwork.hpp"

class DijkstraAoN: public AllOrNothing {
   public:
    virtual StaticSolutionBase solve(
        const StaticNetwork &supply,
        const StaticDemand &demand,
        const StaticSolution &x0 = StaticSolutionBase()
    );
};
