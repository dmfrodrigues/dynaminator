#include "convex/IntervalSolver.hpp"

using namespace std;

typedef IntervalSolver::Var Var;

Var IntervalSolver::solve(){
    pair<Var, Var> p = solveInterval();
    return (p.first+p.second)/2.0;
}
