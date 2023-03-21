#pragma once

#include <functional>

class ConvexSolver {
public:
    typedef double Var;
    typedef std::function<Var(Var)> Problem;
    virtual void setProblem(Problem p) = 0;
    virtual Var solve() = 0;
    virtual ~ConvexSolver(){}
};
