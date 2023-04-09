#pragma once

#include "static/StaticDemand.hpp"
#include "static/supply/StaticNetwork.hpp"

// Case 1 - Sheffi (1985), problem 1.2. (p. 25), solving for UE
StaticNetwork *getStaticNetworkTestCase1();
std::pair<StaticNetwork *, StaticDemand *> getStaticProblemTestCase1();

// Case 2 - Boyles (2019), figure 5.3. (p. 133), solving for UE
StaticNetwork *getStaticNetworkTestCase2();
std::pair<StaticNetwork *, StaticDemand *> getStaticProblemTestCase2();

// Case 3 - Boyles (2019), figure 5.3. (p. 133), solving for SO
StaticNetwork *getStaticNetworkTestCase3();
std::pair<StaticNetwork *, StaticDemand *> getStaticProblemTestCase3();
