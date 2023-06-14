#pragma once

#include "Opt/GeneticSolver.hpp"
#include "Opt/UnivariateSolver.hpp"

namespace Opt {
class GeneticIntervalSolver: public UnivariateSolver {
    GeneticSolver solver;

    Var left, right;

    size_t initialPopulationSize;

   public:
    // clang-format off
    GeneticIntervalSolver(
        size_t populationSize       = 8,
        size_t newPopulationSize    = 8,
        Var    variabilityCoeff     = 0.1,
        size_t maxNumberGenerations = 1000,
        int    parallelism          = 8,
        std::shared_ptr<std::mt19937> gen = std::make_shared<std::mt19937>(std::random_device()())
    );
    // clang-format on

    void         setInterval(Var l, Var r);
    virtual void setStopCriteria(Var e);

    virtual Var solve(Problem p);
};
}  // namespace Opt
