#pragma once

#include "opt/UnivariateSolver.hpp"

class SolverWithInitialSolutions: public UnivariateSolver {
   public:
    virtual void addInitialSolution(Var v) = 0;
    
    template<typename... Args>
    void addInitialSolutions(Args... args){
        (addInitialSolution(args), ...);
    }

    virtual void clearInitialSolutions() = 0;
};
