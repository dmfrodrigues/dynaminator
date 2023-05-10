#pragma once

#include "Opt/GeneticSolver.hpp"
#include "Opt/UnivariateSolver.hpp"

namespace Opt {
class GeneticIntervalSolver: public UnivariateSolver {
    GeneticSolver solver;

    Var left, right;
    size_t initialPopulationSize;

   public:
    GeneticIntervalSolver(
        size_t populationSize       = 100,
        Var    variabilityCoeff     = 0.1,
        size_t tournamentSize       = 1000,
        size_t maxNumberGenerations = 100
    );

    void setInterval(Var l, Var r);
    virtual void setStopCriteria(Var e);

    virtual Var solve(Problem p);
};
}  // namespace Opt
