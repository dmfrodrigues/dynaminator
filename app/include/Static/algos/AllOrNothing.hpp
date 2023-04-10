#pragma once

#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/supply/Network.hpp"

namespace Static {
class AllOrNothing {
   public:
    virtual SolutionBase solve(
        const Network  &supply,
        const Demand   &demand,
        const Solution &flows = SolutionBase()
    ) = 0;
};
}  // namespace Static
