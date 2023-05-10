#include "Opt/GeneticIntervalSolver.hpp"

using namespace std;
using namespace Opt;

typedef UnivariateSolver::Var Var;

GeneticIntervalSolver::GeneticIntervalSolver(
    size_t populationSize_,
    Var    variabilityCoeff_,
    size_t tournamentSize_,
    size_t maxNumberGenerations_
):
    solver(
        populationSize_, 
        variabilityCoeff_, 
        tournamentSize_, 
        maxNumberGenerations_
    ),
    initialPopulationSize(populationSize_) {}

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

    return solver.solve(p);
}
