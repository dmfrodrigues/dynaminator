#pragma once

#include "StaticDemand.hpp"
#include "supply/StaticNetwork.hpp"

struct StaticProblem {
    StaticNetwork &supply;
    StaticDemand demand;
};
