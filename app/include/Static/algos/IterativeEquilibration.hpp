#pragma once

#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/FrankWolfe.hpp"

namespace Static {
template<typename NetworkType>
class IterativeEquilibration {
    FrankWolfe &fw;
    
    double epsilon;
    size_t iterations = 1000;

   public:
    IterativeEquilibration(
        FrankWolfe &fw_
    ) : fw(fw_) {}

    void setIterations(size_t it) {
        iterations = it;
    }

    void setStopCriteria(double stopCriteria){
        epsilon = stopCriteria;
    }

    Solution solve(
        const NetworkType &network,
        const Demand      &demand,
        const Solution    &startingSolution
    ) {
        Solution x = startingSolution;
        for (size_t i = 0; i < iterations; i++) {
            Solution xPrev = x;
            x = fw.solve(
                network.makeConvex(x),
                demand,
                x
            );
        }
        return x;
    }
};
}  // namespace Static
