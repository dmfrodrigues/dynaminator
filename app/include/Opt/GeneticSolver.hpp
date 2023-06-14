#pragma once

#include <memory>
#include <random>

#include "Opt/SolverWithInitialSolutions.hpp"
#include "ctpl_stl.h"

namespace Opt {
class GeneticSolver: public SolverWithInitialSolutions {
    const size_t populationSize;
    const size_t newPopulationSize;
    const Var    variabilityCoeff;
    const size_t maxNumberGenerations;

    std::shared_ptr<std::mt19937> gen;

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
    // clang-format off
    GeneticSolver(
        size_t populationSize,
        size_t newPopulationSize,
        Var    variabilityCoeff,
        size_t maxNumberGenerations,
        int    parallelism = 8,
        std::shared_ptr<std::mt19937> gen = std::make_shared<std::mt19937>(std::random_device()())
    );
    // clang-format on

    virtual void clearInitialSolutions();
    virtual void addInitialSolution(Var v);

    virtual void setStopCriteria(Var e);
    virtual Var  solve(Problem p);
};
}  // namespace Opt
