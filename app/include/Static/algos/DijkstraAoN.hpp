#pragma once

#include "Static/Demand.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/supply/Network.hpp"

namespace Static {
class DijkstraAoN: public AllOrNothing {
   public:
    virtual SolutionBase solve(
        const Network  &supply,
        const Demand   &demand,
        const Solution &x0 = SolutionBase()
    );
};
}  // namespace Static
