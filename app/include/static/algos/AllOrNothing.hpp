#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class AllOrNothing {
    const StaticProblem &problem;

   public:
    AllOrNothing(const StaticProblem &prob);

    StaticSolution solve();
};
