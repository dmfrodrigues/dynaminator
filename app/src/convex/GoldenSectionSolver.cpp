#include "convex/GoldenSectionSolver.hpp"

using namespace std;

void GoldenSectionSolver::setProblem(Problem p){
    f = p;
}

void GoldenSectionSolver::setInterval(double left, double right){
    l = left;
    r = right;
}

void GoldenSectionSolver::setStopCriteria(double e){
    epsilon = e;
}

pair<double, double> GoldenSectionSolver::solveInterval(){
    double delta;
    double a = r-(r-l)*GOLDEN_SECTION;
    double b = l+(r-l)*GOLDEN_SECTION;
    double ya = f(a);
    double yb = f(b);
    while(b-l > epsilon){
        double lprev = l, rprev = r;
        if(ya <= yb){
            r = b;
            b = a; yb = ya;
            a = r-(r-l)*GOLDEN_SECTION; ya = f(a);
        } else {
            l = a;
            a = b; ya = yb;
            b = l+(r-l)*GOLDEN_SECTION; yb = f(b);
        }
        if(l == lprev && r == rprev)
            break;
    }

    if(ya <= yb){
        r = b;
    } else {
        l = a;
    }

    return make_pair(l, r);
}
