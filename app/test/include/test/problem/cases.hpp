#pragma once

#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"

// Case 1 - Sheffi (1985), problem 1.2. (p. 25), solving for UE
Static::Network *getStaticNetworkTestCase1();
std::pair<Static::Network *, Static::Demand *> getStaticProblemTestCase1();

// Case 2 - Boyles (2019), figure 5.3. (p. 133), solving for UE
Static::Network *getStaticNetworkTestCase2();
std::pair<Static::Network *, Static::Demand *> getStaticProblemTestCase2();

// Case 3 - Boyles (2019), figure 5.3. (p. 133), solving for SO
Static::Network *getStaticNetworkTestCase3();
std::pair<Static::Network *, Static::Demand *> getStaticProblemTestCase3();
