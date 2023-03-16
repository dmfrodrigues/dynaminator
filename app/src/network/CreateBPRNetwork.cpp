#include "network/CreateBPRNetwork.hpp"

#include "HttpStatusCodes_C++.h"
#include "data/SumoNetwork.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "utils/serialize.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

CreateBPRNetwork::CreateBPRNetwork() {}

CreateBPRNetwork::CreateBPRNetwork(
    const ResourceId &resourceId_,
    const string &netPath_,
    const string &tazPath_) : resourceId(resourceId_),
                              netPath(netPath_),
                              tazPath(tazPath_) {}

void CreateBPRNetwork::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(resourceId)
        << utils::serialize<string>(netPath)
        << utils::serialize<string>(tazPath);
}

bool CreateBPRNetwork::deserializeContents(stringstream &ss) {
    ss >> utils::deserialize<string>(resourceId) >> utils::deserialize<string>(netPath) >> utils::deserialize<string>(tazPath);
    return (bool)ss;
}

CreateBPRNetwork::Response *CreateBPRNetwork::process() {
    CreateBPRNetwork::Response *res = new CreateBPRNetwork::Response();
    
    try {
        SumoNetwork sumoNetwork = SumoNetwork::loadFromFile(netPath);
        SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(tazPath);

        auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
        StaticNetwork *network = get<0>(t);
        SumoAdapterStatic adapter = get<1>(t);

        if (GlobalState::staticNetworks.count(resourceId)) {
            res->setStatusCode(400);
            res->setReason("Static network with id " + resourceId + " already exists");
            return res;
        }
        GlobalState::staticNetworks[resourceId] = pair<StaticNetwork *, SumoAdapterStatic>(network, adapter);
    } catch(const ios_base::failure &e) {
        res->setStatusCode(400);
        res->setReason("what(): " + string(e.what()) + " (code " + to_string(e.code().value()) + ")");
    } catch (const exception &e) {
        res->setStatusCode(500);
        res->setReason("what(): " + string(e.what()));
    }
    return res;
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork)

void CreateBPRNetwork::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<int>(getStatusCode()) << utils::serialize<string>(getReason());
}

bool CreateBPRNetwork::Response::deserializeContents(stringstream &ss) {
    int s;
    ss >> utils::deserialize<int>(s);
    setStatusCode(s);
    string r;
    ss >> utils::deserialize<string>(r);
    setReason(r);
    return true;
}

void CreateBPRNetwork::Response::handle(ostream &os) {
    os << "Status: " << getStatusCode() << "\n";
    if(getStatusCode() == 200){
        os << "Content-type: application/json\n\n";
    } else {
        os << "Content-type: text/html\n\n";
        os << getReason() << "\n";
    }
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork::Response)
