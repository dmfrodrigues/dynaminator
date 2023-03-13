#include "convex/GoldenSectionSolver.hpp"

using namespace std;

void GoldenSectionSolver::setProblem(Problem p){
    f = p;
}

void GoldenSectionSolver::setInterval(double left_, double right_){
    left = left_;
    right = right_;
}

void GoldenSectionSolver::setStopCriteria(double e){
    epsilon = e;
}

pair<double, double> GoldenSectionSolver::solveInterval(){
    double l = left, r = right;

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
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wfloat-equal"
        if(l == lprev && r == rprev)
            break;
        #pragma GCC diagnostic pop
    }

    if(ya <= yb){
        r = b;
    } else {
        l = a;
    }

    return make_pair(l, r);
}
