#pragma once

#include <list>
#include <vector>

#include "convex/ConvexSolver.hpp"

class QuadraticSolver: public ConvexSolver {
    std::list<Var> initialSols;
    std::vector<std::pair<Var, Var>> sols;
    Problem f;
    Var epsilon;

   public:
    virtual void setSolutions(Var x1, Var x2, Var x3);
    virtual void setProblem(Problem p);
    virtual void setStopCriteria(Var e);
    virtual Var solve();
};
