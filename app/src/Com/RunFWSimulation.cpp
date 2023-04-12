#include "Com/RunFWSimulation.hpp"

#include <HttpStatusCodes_C++.h>
#include <memory>

#include "GlobalState.hpp"
#include "Log/ProgressLoggerJsonOStream.hpp"
#include "Log/ProgressLoggerTableOStream.hpp"
#include "Opt/QuadraticGuessSolver.hpp"
#include "Opt/QuadraticSolver.hpp"
#include "Static/algos/DijkstraAoN.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/TAZ.hpp"

using namespace std;
using namespace Com;
namespace us = utils::serialize;

using ResourceId = GlobalState::ResourceID;

RunFWSimulation::RunFWSimulation() {}

RunFWSimulation::RunFWSimulation(
    const string &netPath_,
    const string &tazPath_,
    const string &demandPath_,
    const string &edgeDataPath_,
    const string &routesPath_
):
    netPath(netPath_),
    tazPath(tazPath_),
    demandPath(demandPath_),
    edgeDataPath(edgeDataPath_),
    routesPath(routesPath_) {}

void RunFWSimulation::serializeContents(stringstream &ss) const {
    ss
        << us::serialize<string>(netPath)
        << us::serialize<string>(tazPath)
        << us::serialize<string>(demandPath)
        << us::serialize<string>(edgeDataPath)
        << us::serialize<string>(routesPath);
}

bool RunFWSimulation::deserializeContents(stringstream &ss) {
    ss
        >> us::deserialize<string>(netPath)
        >> us::deserialize<string>(tazPath)
        >> us::deserialize<string>(demandPath)
        >> us::deserialize<string>(edgeDataPath)
        >> us::deserialize<string>(routesPath);
    return (bool)ss;
}

RunFWSimulation::Response *RunFWSimulation::process() {
    RunFWSimulation::Response *res = new RunFWSimulation::Response();
    try {
        // Create stringstream resource
        GlobalState::ResourceID resourceID = "/static/simulation/"s + "hello" + "/log";
        auto [it, success] = GlobalState::streams->emplace(resourceID, make_shared<stringstream>());
        if(!success){
            res->setStatusCode(400);
            res->setReason("Resource already exists");
        }
        std::stringstream &ss = *it->second;
        Log::ProgressLoggerJsonOStream loggerOStream(ss);
        Log::ProgressLogger &logger = loggerOStream;

        // Supply
        SUMO::Network            sumoNetwork = SUMO::Network::loadFromFile(netPath);
        SUMO::TAZs               sumoTAZs    = SUMO::TAZ::loadFromFile(tazPath);
        auto                     t           = Static::BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        Static::BPRNetwork      *network     = get<0>(t);
        const SumoAdapterStatic &adapter     = get<1>(t);

        // Demand
        VISUM::OFormatDemand oDemand = VISUM::OFormatDemand::loadFromFile(demandPath);
        Static::Demand       demand  = Static::Demand::fromOFormat(oDemand, adapter);

        // Solve

        // All or Nothing
        Static::DijkstraAoN aon;
        Static::Solution    x0 = aon.solve(*network, demand);

        // Solver
        Opt::QuadraticSolver      innerSolver;
        Opt::QuadraticGuessSolver solver(
            innerSolver,
            0.5,
            0.2,
            0.845,
            0.365
        );
        solver.setStopCriteria(0.01);

        // Frank-Wolfe
        Static::FrankWolfe fw(aon, solver, logger);
        fw.setStopCriteria(1.0);
        Static::Solution x = fw.solve(*network, demand, x0);

        network->saveResultsToFile(x, adapter, edgeDataPath, routesPath);

        return res;
    } catch(const exception &e) {
        res->setStatusCode(500);
        res->setReason("what(): " + string(e.what()));
        return res;
    }
}

MESSAGE_REGISTER_DEF(Com::RunFWSimulation)

void RunFWSimulation::Response::serializeContents(stringstream &ss) const {
    ss
        << us::serialize<int>(getStatusCode())
        << us::serialize<string>(getReason());
}

bool RunFWSimulation::Response::deserializeContents(stringstream &ss) {
    int s;
    ss >> us::deserialize<int>(s);
    setStatusCode(s);
    string r;
    ss >> us::deserialize<string>(r);
    setReason(r);
    return true;
}

void RunFWSimulation::Response::handle(ostream &os) {
    if(getStatusCode() == 200)
        os << "Content-type: application/json\n\n";
    else {
        os << "Content-type: text/html\n";
        os << "Status: " << getStatusCode() << " "
           << HttpStatus::reasonPhrase(getStatusCode())
           << "\n\n";
        os << getReason() << "\n";
    }
}

MESSAGE_REGISTER_DEF(Com::RunFWSimulation::Response)
