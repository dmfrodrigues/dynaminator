#pragma once

#include "static/supply/StaticNetwork.hpp"
#include "static/StaticProblem.hpp"

// Case 1 - Sheffi (1985), problem 1.2. (p. 25)
StaticNetwork *getStaticNetworkTestCase1();
StaticProblem *getStaticProblemTestCase1();

// Case 2 - Boyles (2019), figure 5.3. (p. 133)
StaticNetwork *getStaticNetworkTestCase2();
StaticProblem *getStaticProblemTestCase2();

// Case 3 - Boyles (2019), figure 5.3. (p. 133), but this time solving for UE
StaticNetwork *getStaticNetworkTestCase3();
StaticProblem *getStaticProblemTestCase3();
