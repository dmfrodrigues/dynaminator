#include "convex/IntervalSolver.hpp"

using namespace std;

double IntervalSolver::solve(){
    pair<double, double> p = solveInterval();
    return (p.first+p.second)/2.0;
}