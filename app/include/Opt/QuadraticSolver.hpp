#pragma once

#include <list>
#include <vector>

#include "Opt/SolverWithInitialSolutions.hpp"
#include "Opt/UnivariateSolver.hpp"

namespace Opt {
class QuadraticSolver: public SolverWithInitialSolutions {
    std::list<Var> initialSols;

    Var epsilon;

   public:
    virtual void addInitialSolution(Var v);
    virtual void clearInitialSolutions();
    virtual void setStopCriteria(Var e);
    virtual Var  solve(Problem p);
};
}  // namespace Opt
