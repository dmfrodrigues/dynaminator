#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class AllOrNothing {
   public:
    virtual StaticSolutionBase solve(
        const StaticProblem &prob,
        const StaticSolution &flows = StaticSolutionBase()
    ) = 0;
};
