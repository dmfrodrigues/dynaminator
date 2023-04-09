#pragma once

#include "opt/UnivariateSolver.hpp"

#include <utility>

class IntervalSolver: public UnivariateSolver {
public:
    virtual void setInterval(Var l, Var r) = 0;
    virtual void setStopCriteria(Var e) = 0;
    virtual std::pair<Var, Var> solveInterval(Problem p) = 0;
    virtual Var solve(Problem p) final;
};
