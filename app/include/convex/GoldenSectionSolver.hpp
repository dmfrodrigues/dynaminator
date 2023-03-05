#pragma once

#include "IntervalSolver.hpp"

#include <utility>

class GoldenSectionSolver: public IntervalSolver {
    static constexpr double GOLDEN_SECTION = 0.61803398875; // 0.5 * (sqrt(5.0) - 1.0)

    Problem f;
    double l, r, epsilon;
public:
    virtual void setProblem(Problem p);
    virtual void setInterval(double l, double r);
    virtual void setStopCriteria(double e);
    virtual std::pair<double, double> solveInterval();
};
