#pragma once

#include "ConvexSolver.hpp"

#include <utility>

class IntervalSolver: public ConvexSolver {
public:
    virtual void setInterval(double l, double r) = 0;
    virtual void setStopCriteria(double e) = 0;
    virtual std::pair<double, double> solveInterval() = 0;
    virtual double solve() final;
};
