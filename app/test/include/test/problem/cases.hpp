#pragma once

#include "static/supply/StaticNetwork.hpp"
#include "static/StaticProblem.hpp"

#include <memory>

std::unique_ptr<StaticNetwork> getStaticNetworkTestCase1();
std::unique_ptr<StaticProblem> getStaticProblemTestCase1();
