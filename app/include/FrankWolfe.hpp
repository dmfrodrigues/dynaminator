#pragma once

#include "StaticProblem.hpp"
#include "StaticSolution.hpp"

class FrankWolfe {

    StaticProblem problem;

    StaticSolution xn;

public:
    void setStartingSolution(StaticSolution startingSolution);

    StaticSolution solve();

private:
    StaticSolution step1();
    StaticSolution step2(const StaticSolution &xstar);

};
