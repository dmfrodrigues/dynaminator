#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>

#include "Opt/QuadraticSolver.hpp"
#include "Opt/UnivariateSolver.hpp"
#include "Opt/GoldenSectionSolver.hpp"

#include <cmath>
#include <iostream>

using namespace std;

using Catch::Matchers::WithinAbs;

void testInterval(
    const Opt::IntervalSolver::Interval &p,
    const Opt::IntervalSolver::Interval &psol
){
    // Check consistency of solution
    REQUIRE(p.first <= p.second);

    // Check range is completely included in range allowed by error around true solution
    REQUIRE(psol.first <= p.first);
    REQUIRE(p.second <= psol.second);
}

void testIntervalSolverQuadratic(
    Opt::IntervalSolver &solver,
    const double &a,
    const double &b,
    const double &c,
    const double &lmargin,
    const double &rmargin,
    const double &e
) {
    double sol = -b / (a * 2);
    Opt::IntervalSolver::Interval psol(sol - e, sol+e);
    double l = sol - lmargin;
    double r = sol + rmargin;
    Opt::UnivariateSolver::Problem prob = [a, b, c](double x){
        return (((a)*x + b)*x + c);
    };
    solver.setInterval(l, r);
    solver.setStopCriteria(e);
    Opt::IntervalSolver::Interval p = solver.solveInterval(prob);

    testInterval(p, psol);
}

void testSolverQuadratic(
    Opt::UnivariateSolver &solver,
    const double &a,
    const double &b,
    const double &c,
    const double &e) {
    double sol = -b / (a * 2);
    Opt::IntervalSolver::Interval psol(sol - e, sol+e);
    Opt::UnivariateSolver::Problem prob = [a, b, c](double x){
        return (((a)*x + b)*x + c);
    };
    solver.setStopCriteria(e);
    double x = solver.solve(prob);

    REQUIRE_THAT(x, WithinAbs(sol, e));
}

TEST_CASE("Golden Section solver", "[golden-section]") {
    Catch::StringMaker<float>::precision = 40;

    Opt::GoldenSectionSolver solver;

    SECTION("Quadratic 1,0,0, error 1e-3, margins 1,1") {
        double a = 1, b = 0, c = 0, e = 1e-3, lmargin = 1, rmargin = 1;
        testIntervalSolverQuadratic(solver, a, b, c, lmargin, rmargin, e);
    }

    SECTION("Quadratic 1,1,1, error 1e-3, margins 10,100") {
        double a = 1, b = 1, c = 1, e = 1e-3, lmargin = 10, rmargin = 100;
        testIntervalSolverQuadratic(solver, a, b, c, lmargin, rmargin, e);
    }

    SECTION("Quadratic 1,0,0, error 1e-7, margins 1,1") {
        double a = 1, b = 0, c = 0, e = 1e-7, lmargin = 1, rmargin = 1;
        testIntervalSolverQuadratic(solver, a, b, c, lmargin, rmargin, e);
    }

    SECTION("Quadratic 1,1,1, error 1e-7, margins 10,100") {
        double a = 1, b = 1, c = 1, e = 1e-7, lmargin = 10, rmargin = 100;
        testIntervalSolverQuadratic(solver, a, b, c, lmargin, rmargin, e);
    }

    SECTION("Quadratic 123,456,789, error 1e-7, margins 10000,10000") {
        double a = 123, b = 456, c = 789, e = 1e-7, lmargin = 10000, rmargin = 10000;
        testIntervalSolverQuadratic(solver, a, b, c, lmargin, rmargin, e);
    }

    SECTION("Absolute value, sol 2, error 1e-7, interval -3,14") {
        double sol = 2, e = 1e-15, l = -3, r = 14;
        Opt::UnivariateSolver::Problem prob = [sol](double x){
            return fabs(x-sol);
        };
        solver.setInterval(l, r);
        solver.setStopCriteria(e);
        Opt::IntervalSolver::Interval p = solver.solveInterval(prob);

        testInterval(p, make_pair(sol-e, sol+e));
    }
}
