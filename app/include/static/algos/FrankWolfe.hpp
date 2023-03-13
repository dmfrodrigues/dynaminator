#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class FrankWolfe {
    StaticProblem problem;

    StaticSolution xn;
    double epsilon;

   public:
    FrankWolfe(StaticProblem prob);

    void setStartingSolution(StaticSolution startingSolution);
    void setStopCriteria(double e);

    StaticSolution solve();

   private:
    StaticSolution step1();
    StaticSolution step2(const StaticSolution &xstar);
};
