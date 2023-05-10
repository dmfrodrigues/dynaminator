#pragma once

#include "Opt/SolverWithInitialSolutions.hpp"

namespace Opt {
class GeneticSolver: public SolverWithInitialSolutions {
    const size_t populationSize;
    const Var    variabilityCoeff;
    const size_t tournamentSize;
    const size_t maxNumberGenerations;

    std::vector<std::pair<Var, Var>> population;

    Var epsilon = 1e-9;

    const Problem *problem = nullptr;

    void crossover();
    Var  crossover(const Var &a, const Var &b);

    void mutation();
    Var &mutation(Var &v);
    Var  getVariability() const;

    void tournament();

   public:
    GeneticSolver(
        size_t populationSize       = 100,
        Var    variabilityCoeff     = 0.1,
        size_t tournamentSize       = 1000,
        size_t maxNumberGenerations = 100
    );

    virtual void clearInitialSolutions();
    virtual void addInitialSolution(Var v);

    virtual void setStopCriteria(Var e);
    virtual Var  solve(Problem p);
};
}  // namespace Opt
