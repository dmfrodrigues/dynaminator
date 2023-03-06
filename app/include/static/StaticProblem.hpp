#pragma once

#include "StaticDemand.hpp"
#include "supply/StaticNetwork.hpp"

struct StaticProblem {
    const StaticNetwork &supply;
    StaticDemand demand;
};
