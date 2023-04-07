#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class AllOrNothing {
    const StaticProblem &problem;
    const StaticSolution &x0;

   public:
    AllOrNothing(const StaticProblem &prob, const StaticSolution &flows = StaticSolutionBase());

    StaticSolutionBase solve();
};
