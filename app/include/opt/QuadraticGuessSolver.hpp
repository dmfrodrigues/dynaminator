#pragma once

#include "opt/SolverWithInitialSolutions.hpp"
#include "opt/UnivariateSolver.hpp"

/**
 * @brief Solver that guesses the value of the next solution.
 *
 * Provides three solutions to the solver. Solutions are calculated as follows:
 * $s_{n+1} = (1-\alpha) s_n + \alpha x_n$
 * $s_1 = M * s_n$
 * $s_2 = s_1 * 10^{-m}$
 * $s_3 = s_1 * 10^{m}$
 * where:
 * - $s_n$ are solution estimates.
 * - $\alpha$ is the exponential smoothing factor; should be a small value,
 *   typically 0.1 or 0.2 to prevent excessive jitter.
 * - $M$ is the multiplicative adjustment factor. This is because a small
 *   $\alpha$ avoids jitter, but this delays adapting to more recent answers.
 *   Since we already know where the process is going (e.g., we have an idea
 *   that the real value is 85% of the estimate), we can adjust the estimation
 *   by multiplying all solutions with $M$.
 * - $m$ is the margin.
 */
class QuadraticGuessSolver: public UnivariateSolver {
    SolverWithInitialSolutions &solver;
    Var s;
    Var alpha;
    Var multAdjust;
    Var margin;

   public:
    /**
     * @brief Construct a new Quadratic Solver Guesser object.
     *
     * @param initialSolution   Initial solution
     * @param alpha             Exponential smoothing factor
     * @param multAdjust        Multiplicative adjustment factor
     * @param margin            Margin (in logarithmic scale, log10)
     */
    QuadraticGuessSolver(
        SolverWithInitialSolutions &solver,
        Var initialSolution,
        Var alpha = 0.2,
        Var multAdjust = 1.0,
        Var margin = 1.0
    );

    virtual void setStopCriteria(Var e);
    
    virtual Var solve(Problem p);
};
