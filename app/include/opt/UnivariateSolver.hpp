#pragma once

#include <functional>

class UnivariateSolver {
public:
    typedef double Var;
    typedef std::function<Var(Var)> Problem;
    virtual Var solve(Problem p) = 0;
    virtual ~UnivariateSolver(){}
};
