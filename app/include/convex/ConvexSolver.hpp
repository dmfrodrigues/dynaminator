#pragma once

#include <functional>

class ConvexSolver {
public:
    using Problem = std::function<double(double)>;
    virtual void setProblem(Problem p) = 0;
    virtual double solve() = 0;
    virtual ~ConvexSolver(){}
};
