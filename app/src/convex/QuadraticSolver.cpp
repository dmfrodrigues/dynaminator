#include "convex/QuadraticSolver.hpp"

#include <cmath>

typedef ConvexSolver::Problem Problem;
typedef ConvexSolver::Var Var;

void QuadraticSolver::setSolutions(Var x1, Var x2, Var x3) {
    x = {x1, x2, x3};
}

void QuadraticSolver::setProblem(Problem p) {
    f = p;
}

void QuadraticSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var QuadraticSolver::solve() {
    for (size_t i = 0; i < x.size(); ++i) {
        z.emplace_back(f(x[i]));
    }

    while(fabs(x[2] - x[1]) > epsilon){
        const Var
            &x1 = x[0],
            &x2 = x[1],
            &x3 = x[2];
        const Var
            xx1 = x1*x1,
            xx2 = x2*x2,
            xx3 = x3*x3;
        const Var
            &z1 = z[0],
            &z2 = z[1],
            &z3 = z[2];

        Var xn = 0.5 * (
            (xx2-xx3)*z1 + (xx3-xx1)*z2 + (xx1-xx2)*z3
        )/(
            ( x2- x3)*z1 + ( x3- x1)*z2 + ( x1- x2)*z3
        );
        Var zn = f(xn);

        x.pop_front(); x.push_back(xn);
        z.pop_front(); z.push_back(zn);
    }

    return x[2];
}
