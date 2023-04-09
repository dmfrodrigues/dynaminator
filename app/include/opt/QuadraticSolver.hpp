#pragma once

#include <list>
#include <vector>

#include "opt/SolverWithInitialSolutions.hpp"
#include "opt/UnivariateSolver.hpp"

class QuadraticSolver: public SolverWithInitialSolutions {
    std::list<Var> initialSols;
    Var epsilon;

   public:
    virtual void addInitialSolution(Var v);
    virtual void setStopCriteria(Var e);
    virtual Var solve(Problem p);
};
