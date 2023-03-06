#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class FrankWolfe {
    StaticProblem problem;

    StaticSolution xn;

   public:
    FrankWolfe(StaticProblem prob);

    void setStartingSolution(StaticSolution startingSolution);

    StaticSolution solve();

   private:
    StaticSolution step1();
    StaticSolution step2(const StaticSolution &xstar);
};
