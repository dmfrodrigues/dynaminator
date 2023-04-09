#pragma once

#include <list>
#include <vector>

#include "opt/UnivariateSolver.hpp"

class QuadraticSolver: public UnivariateSolver {
    std::list<Var> initialSols;
    std::vector<std::pair<Var, Var>> sols;
    Var epsilon;

   public:
    virtual void setSolutions(Var x1, Var x2, Var x3);
    virtual void setStopCriteria(Var e);
    virtual Var solve(Problem p);
};
