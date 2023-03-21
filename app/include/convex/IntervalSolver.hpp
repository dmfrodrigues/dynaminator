#pragma once

#include "ConvexSolver.hpp"

#include <utility>

class IntervalSolver: public ConvexSolver {
public:
    virtual void setInterval(Var l, Var r) = 0;
    virtual void setStopCriteria(Var e) = 0;
    virtual std::pair<Var, Var> solveInterval() = 0;
    virtual Var solve() final;
};
