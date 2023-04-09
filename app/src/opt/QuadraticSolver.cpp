#include "opt/QuadraticSolver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

typedef UnivariateSolver::Problem Problem;
typedef UnivariateSolver::Var Var;

void QuadraticSolver::addInitialSolution(Var v) {
    initialSols.push_back(v);
}

void QuadraticSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var QuadraticSolver::solve(Problem f) {
    if(initialSols.size() < 3){
        throw logic_error("QuadraticSolver requires at least 3 initial solutions");
    }

    std::vector<std::pair<Var, Var>> sols;
    sols.reserve(initialSols.size());

    for(const Var &x: initialSols)
        sols.emplace_back(f(x), x);

    while(sols.size() > 3)
        sols.pop_back();
    sort(sols.begin(), sols.end());

    sols.reserve(4);

    Var xPrev = sols.begin()->second;
    Var x = sols.rbegin()->second;

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
        while(sols.size() > 3)
            sols.pop_back();
    }

    initialSols.clear();

    return x;
}
