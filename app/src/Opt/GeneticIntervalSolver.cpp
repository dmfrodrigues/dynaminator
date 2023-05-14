#include "Opt/GeneticIntervalSolver.hpp"

using namespace std;
using namespace Opt;

typedef UnivariateSolver::Var Var;

GeneticIntervalSolver::GeneticIntervalSolver(
    size_t populationSize,
    size_t newPopulationSize,
    Var    variabilityCoeff,
    size_t maxNumberGenerations,
    int parallelism
):
    solver(
        populationSize, 
        newPopulationSize,
        variabilityCoeff,
        maxNumberGenerations,
        parallelism
    ),
    initialPopulationSize(populationSize + newPopulationSize) {}

void GeneticIntervalSolver::setInterval(Var l, Var r) {
    left  = l;
    right = r;
}

void GeneticIntervalSolver::setStopCriteria(Var e) {
    solver.setStopCriteria(e);
}

Var GeneticIntervalSolver::solve(Problem p) {
    solver.clearInitialSolutions();

    for(size_t i = 0; i < initialPopulationSize; ++i){
        double r = (double)i/((double)initialPopulationSize - 1.0);
        solver.addInitialSolution(left + (right - left) * r);
    }

    Problem p_ = [&p, this](Var a) -> Var {
        if(a < left || a > right) return numeric_limits<Var>::infinity();
        return p(a);
    };

    return solver.solve(p_);
}
