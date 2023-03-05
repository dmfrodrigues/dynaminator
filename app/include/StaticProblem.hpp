#pragma once

#include "StaticDemand.hpp"
#include "StaticNetwork.hpp"

struct StaticProblem {
    StaticNetwork &supply;
    StaticDemand demand;
};
