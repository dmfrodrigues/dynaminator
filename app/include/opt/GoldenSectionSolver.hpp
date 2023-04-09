#pragma once

#include "IntervalSolver.hpp"

#include <utility>

class GoldenSectionSolver: public IntervalSolver {
    static constexpr Var GOLDEN_SECTION = 0.61803398875; // 0.5 * (sqrt(5.0) - 1.0)

    Var left, right, epsilon;
public:
    virtual void setInterval(Var l, Var r);
    virtual void setStopCriteria(Var e);
    virtual std::pair<Var, Var> solveInterval(Problem p);
};
