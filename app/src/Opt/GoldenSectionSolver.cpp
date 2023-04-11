#include "Opt/GoldenSectionSolver.hpp"

using namespace std;
using namespace Opt;

typedef GoldenSectionSolver::Var Var;

void GoldenSectionSolver::setInterval(Var left_, Var right_){
    left = left_;
    right = right_;
}

void GoldenSectionSolver::setStopCriteria(Var e){
    epsilon = e;
}

pair<Var, Var> GoldenSectionSolver::solveInterval(Problem f){
    Var l = left, r = right;

    Var a = r-(r-l)*GOLDEN_SECTION;
    Var b = l+(r-l)*GOLDEN_SECTION;
    Var ya = f(a);
    Var yb = f(b);
    while(b-l > epsilon){
        Var lprev = l, rprev = r;
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
