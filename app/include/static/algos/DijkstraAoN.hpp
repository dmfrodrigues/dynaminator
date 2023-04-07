#pragma once

#include "static/algos/AllOrNothing.hpp"

class DijkstraAoN: public AllOrNothing {
   public:
    virtual StaticSolutionBase solve(
        const StaticProblem &problem,
        const StaticSolution &x0 = StaticSolutionBase()
    );
};
