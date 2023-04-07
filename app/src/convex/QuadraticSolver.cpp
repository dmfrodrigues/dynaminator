#include "convex/QuadraticSolver.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

typedef ConvexSolver::Problem Problem;
typedef ConvexSolver::Var Var;

void QuadraticSolver::setSolutions(Var x1, Var x2, Var x3) {
    initialSols = {x1, x2, x3};
}

void QuadraticSolver::setProblem(Problem p) {
    f = p;
}

void QuadraticSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var QuadraticSolver::solve() {
    sols.clear();
    sols.reserve(4);

    for(const Var &x: initialSols) {
        sols.emplace_back(f(x), x);
    }

    Var xPrev = *min_element(initialSols.begin(), initialSols.end());
    Var x = *max_element(initialSols.begin(), initialSols.end());

    while(fabs(xPrev - x) > epsilon) {
        xPrev = x;

        const Var
            &x1 = sols[0].second,
            &x2 = sols[1].second,
            &x3 = sols[2].second;
        const Var
            xx1 = x1*x1,
            xx2 = x2*x2,
            xx3 = x3*x3;
        const Var
            &z1 = sols[0].first,
            &z2 = sols[1].first,
            &z3 = sols[2].first;

        x = 0.5 * (
            (xx2-xx3)*z1 + (xx3-xx1)*z2 + (xx1-xx2)*z3
        )/(
            ( x2- x3)*z1 + ( x3- x1)*z2 + ( x1- x2)*z3
        );

        sols.emplace_back(f(x), x);
        sort(sols.begin(), sols.end());
        sols.pop_back();
    }

    return x;
}
