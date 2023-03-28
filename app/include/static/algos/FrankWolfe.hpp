#pragma once

#include "static/StaticProblem.hpp"
#include "static/StaticSolution.hpp"

class FrankWolfe {
    StaticProblem problem;

    StaticSolution xn;
    StaticNetwork::Cost epsilon;
    int iterations = 1000;

   public:
    FrankWolfe(StaticProblem prob);

    void setStartingSolution(StaticSolution startingSolution);
    void setStopCriteria(StaticNetwork::Cost e);
    void setIterations(int it);

    StaticSolution solve();

   private:
    StaticSolution step1();
    StaticSolution step2(const StaticSolution &xstar);
};
