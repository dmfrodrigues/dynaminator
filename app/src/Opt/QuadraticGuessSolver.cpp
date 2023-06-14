#include "Opt/QuadraticGuessSolver.hpp"

#include <cmath>
#include <iostream>

using namespace std;
using namespace Opt;

typedef UnivariateSolver::Var Var;

QuadraticGuessSolver::QuadraticGuessSolver(
    SolverWithInitialSolutions &solver_,
    Var                         initialSolution_,
    Var                         alpha_,
    Var                         multAdjust_,
    Var                         margin_
):
    solver(solver_),
    s(initialSolution_),
    alpha(alpha_),
    multAdjust(multAdjust_),
    margin(margin_) {}

void QuadraticGuessSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var QuadraticGuessSolver::solve(Problem prob) {
    Var p  = pow(10, margin);
    Var s1 = s * multAdjust;
    Var s2 = s * p;
    Var s3 = s / p;
    Var e  = fabs(s2 - s3) * epsilon;
    solver.clearInitialSolutions();
    solver.addInitialSolutions(s1, s2, s3);
    solver.setStopCriteria(e);
    Var x = solver.solve(prob);
    s     = (1.0 - alpha) * s + alpha * x;
    return x;
}
