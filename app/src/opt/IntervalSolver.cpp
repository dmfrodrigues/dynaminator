#include "opt/IntervalSolver.hpp"

using namespace std;

typedef IntervalSolver::Var Var;

Var IntervalSolver::solve(Problem prob){
    pair<Var, Var> p = solveInterval(prob);
    return (p.first+p.second)/2.0;
}
