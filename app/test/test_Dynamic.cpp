#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>

#include "Dynamic/Environment.hpp"
#include "Dynamic/Environment_Loader.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"

using namespace std;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

extern string baseDir;

typedef chrono::steady_clock clk;

const size_t MATRIX_9_10_TOTAL_DEMAND_HOUR = 102731;

TEST_CASE("Dynamic environment", "[dynamic]") {
    Log::ProgressLoggerTableOStream logger;

    Dynamic::Environment::Loader<const SUMO::NetworkTAZs &> loader;

    // Supply
    SUMO::Network     sumoNetwork = SUMO::Network::loadFromFile(baseDir + "data/porto/porto.net.xml");
    SUMO::TAZs        sumoTAZs    = SUMO::TAZ::loadFromFile(baseDir + "data/porto/porto.taz.xml");
    SUMO::NetworkTAZs sumo{sumoNetwork, sumoTAZs};

    Dynamic::Environment *env = loader.load(sumo);

    // Demand
    VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(baseDir + "data/porto/matrix.9.0.10.0.2.fma");
    // clang-format off
    Static::Demand::Loader<
        const VISUM::OFormatDemand &,
        const Static::SUMOAdapter &
    > staticDemandLoader;
    // clang-format on
    Static::Demand staticDemand = staticDemandLoader.load(
        oDemand,
        (Static::SUMOAdapter &)loader.adapter
    );

    Dynamic::Demand::UniformLoader demandLoader(1.0, 0.0, 3600.0);
    Dynamic::Demand                demand = demandLoader.load(staticDemand);

    REQUIRE(MATRIX_9_10_TOTAL_DEMAND_HOUR == demand.getVehicles().size());

    delete env;
}
