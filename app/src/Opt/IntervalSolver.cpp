#include "Opt/IntervalSolver.hpp"

using namespace std;
using namespace Opt;

typedef IntervalSolver::Var Var;

Var IntervalSolver::solve(Problem prob) {
    const auto &[l, r] = solveInterval(prob);
    return (l + r) / 2.0;
}
