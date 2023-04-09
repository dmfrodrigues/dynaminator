#include "opt/QuadraticGuessSolver.hpp"

#include <cmath>

typedef UnivariateSolver::Var Var;

QuadraticGuessSolver::QuadraticGuessSolver(
    SolverWithInitialSolutions &solver_,
    Var initialSolution_,
    Var alpha_,
    Var multAdjust_,
    Var margin_
):
    solver(solver_),
    s(initialSolution_),
    alpha(alpha_),
    multAdjust(multAdjust_),
    margin(margin_) {}

void QuadraticGuessSolver::setStopCriteria(Var e){
    solver.setStopCriteria(e);
}

Var QuadraticGuessSolver::solve(Problem prob) {
    Var p = pow(10, margin);
    Var s1 = s * multAdjust;
    Var s2 = s * p;
    Var s3 = s / p;
    solver.clearInitialSolutions();
    solver.addInitialSolutions(s1, s2, s3);
    Var x = solver.solve(prob);
    s = (1.0 - alpha) * s + alpha * x;
    return x;
}
