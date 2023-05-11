#pragma once

#include "Opt/SolverWithInitialSolutions.hpp"
#include "ctpl_stl.h"

namespace Opt {
class GeneticSolver: public SolverWithInitialSolutions {
    const size_t populationSize;
    const size_t newPopulationSize;
    const Var    variabilityCoeff;
    const size_t maxNumberGenerations;

    std::vector<std::pair<Var, Var>> population;
    std::vector<Var> newPopulation;

    Var epsilon = 1e-9;

    const Problem *problem = nullptr;

    ctpl::thread_pool pool;

    void crossover();
    Var  crossover(const Var &a, const Var &b);

    void mutation();
    Var &mutation(Var &v, const Var &variability);
    Var  getVariability() const;

    void tournament();

   public:
    GeneticSolver(
        size_t populationSize,
        size_t newPopulationSize,
        Var    variabilityCoeff,
        size_t maxNumberGenerations,
        int parallelism = 8
    );

    virtual void clearInitialSolutions();
    virtual void addInitialSolution(Var v);

    virtual void setStopCriteria(Var e);
    virtual Var  solve(Problem p);
};
}  // namespace Opt
