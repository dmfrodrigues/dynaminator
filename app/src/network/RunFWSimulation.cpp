#include "network/RunFWSimulation.hpp"

#include "static/algos/AllOrNothing.hpp"
#include "static/algos/FrankWolfe.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

RunFWSimulation::RunFWSimulation() {}

RunFWSimulation::RunFWSimulation(
    const ResourceId &networkId_,
    const ResourceId &demandId_,
    const string &outPath_) : networkId(networkId_),
                              demandId(demandId_),
                              outPath(outPath_) {}

void RunFWSimulation::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(networkId)
        << utils::serialize<string>(demandId)
        << utils::serialize<string>(outPath);
}

bool RunFWSimulation::deserializeContents(stringstream &ss) {
    ss >> utils::deserialize<string>(networkId) >> utils::deserialize<string>(demandId) >> utils::deserialize<string>(outPath);
    return (bool)ss;
}

RunFWSimulation::Response *RunFWSimulation::process() {
    RunFWSimulation::Response *res = new RunFWSimulation::Response();
    try {
        const StaticNetwork *network = GlobalState::staticNetworks.at(networkId).first;
        const SumoAdapterStatic &adapter = GlobalState::staticNetworks.at(networkId).second;
        const StaticDemand &demand = GlobalState::staticDemands.at(demandId);

        StaticProblem problem{*network, demand};

        AllOrNothing aon(problem);
        StaticSolution x0 = aon.solve();

        FrankWolfe fw(problem);
        fw.setStartingSolution(x0);
        fw.setStopCriteria(1e-7);
        StaticSolution x = fw.solve();

        network->saveResultsToFile(x, adapter, outPath);

        return res;
    } catch (const exception &e) {
        res->setSuccess(false);
        return res;
    }
}

MESSAGE_REGISTER_DEF(RunFWSimulation)

void RunFWSimulation::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<bool>(getSuccess());
}

bool RunFWSimulation::Response::deserializeContents(stringstream &ss) {
    bool s;
    ss >> utils::deserialize<bool>(s);
    setSuccess(s);
    return true;
}

void RunFWSimulation::Response::handle(ostream &os) {
    os << "Content-type: application/json\n\n";
}

MESSAGE_REGISTER_DEF(RunFWSimulation::Response)
