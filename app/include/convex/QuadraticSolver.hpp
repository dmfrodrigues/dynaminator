#pragma once

#include "convex/ConvexSolver.hpp"

#include <functional>
#include <deque>

class QuadraticSolver : public ConvexSolver {
    std::deque<Var> x;
    std::deque<Var> z;
    Problem f;
    Var epsilon;
public:
    virtual void setSolutions(Var x1, Var x2, Var x3);
    virtual void setProblem(Problem p);
    virtual void setStopCriteria(Var e);
    virtual Var solve();
};
