#include "network/RunFWSimulation.hpp"

#include "HttpStatusCodes_C++.h"
#include "data/sumo/TAZs.hpp"
#include "static/algos/AllOrNothing.hpp"
#include "static/algos/FrankWolfe.hpp"
#include "static/supply/BPRNetwork.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

RunFWSimulation::RunFWSimulation() {}

RunFWSimulation::RunFWSimulation(
    const string &netPath_,
    const string &tazPath_,
    const string &demandPath_,
    const string &outPath_) : netPath(netPath_),
                              tazPath(tazPath_),
                              demandPath(demandPath_),
                              outPath(outPath_) {}

void RunFWSimulation::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(netPath)
        << utils::serialize<string>(tazPath)
        << utils::serialize<string>(demandPath)
        << utils::serialize<string>(outPath);
}

bool RunFWSimulation::deserializeContents(stringstream &ss) {
    ss
        >> utils::deserialize<string>(netPath)
        >> utils::deserialize<string>(tazPath)
        >> utils::deserialize<string>(demandPath)
        >> utils::deserialize<string>(outPath);
    return (bool)ss;
}

RunFWSimulation::Response *RunFWSimulation::process() {
    RunFWSimulation::Response *res = new RunFWSimulation::Response();
    try {
        // Supply
        SUMO::Network sumoNetwork = SUMO::Network::loadFromFile(netPath);
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(tazPath);
        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        BPRNetwork *network = get<0>(t);
        const SumoAdapterStatic &adapter = get<1>(t);

        // Demand
        OFormatDemand oDemand = OFormatDemand::loadFromFile(demandPath);
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        // Solve
        StaticProblem problem{*network, demand};

        AllOrNothing aon(problem);
        StaticSolution x0 = aon.solve();

        FrankWolfe fw(problem);
        fw.setStartingSolution(x0);
        fw.setStopCriteria(1e-1);
        StaticSolution x = fw.solve();

        network->saveResultsToFile(x, adapter, outPath);

        return res;
    } catch (const exception &e) {
        res->setStatusCode(500);
        res->setReason("what(): " + string(e.what()));
        return res;
    }
}

MESSAGE_REGISTER_DEF(RunFWSimulation)

void RunFWSimulation::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<int>(getStatusCode()) << utils::serialize<string>(getReason());
}

bool RunFWSimulation::Response::deserializeContents(stringstream &ss) {
    int s;
    ss >> utils::deserialize<int>(s);
    setStatusCode(s);
    string r;
    ss >> utils::deserialize<string>(r);
    setReason(r);
    return true;
}

void RunFWSimulation::Response::handle(ostream &os) {
    if (getStatusCode() == 200)
        os << "Content-type: application/json\n\n";
    else
        os << "Status: " << getStatusCode() << " " << HttpStatus::reasonPhrase(getStatusCode()) << "\n";
}

MESSAGE_REGISTER_DEF(RunFWSimulation::Response)
