#include "network/CreateBPRNetwork.hpp"

#include "data/SumoNetwork.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "utils/serialize.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

CreateBPRNetwork::CreateBPRNetwork() {}

CreateBPRNetwork::CreateBPRNetwork(
    const ResourceId &resourceId_,
    const string &netPath_,
    const string &tazPath_
) :
    resourceId(resourceId_),
    netPath(netPath_),
    tazPath(tazPath_)
{}

void CreateBPRNetwork::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(resourceId)
        << utils::serialize<string>(netPath)
        << utils::serialize<string>(tazPath);
}

bool CreateBPRNetwork::deserializeContents(stringstream &ss) {
    ss
        >> utils::deserialize<string>(resourceId)
        >> utils::deserialize<string>(netPath)
        >> utils::deserialize<string>(tazPath);
    return (bool)ss;
}

CreateBPRNetwork::Response *CreateBPRNetwork::process() {
    SumoNetwork sumoNetwork = SumoNetwork::loadFromFile(netPath);
    SumoTAZs sumoTAZs = SumoTAZs::loadFromFile(tazPath);
    
    auto t = BPRNetwork::fromSumo(sumoNetwork, sumoTAZs);
    StaticNetwork *network = get<0>(t);
    SumoAdapterStatic adapter = get<1>(t);

    CreateBPRNetwork::Response *res = new CreateBPRNetwork::Response();
    if(GlobalState::staticNetworks.count(resourceId)){
        res->setSuccess(false);
        return res;
    }
    GlobalState::staticNetworks[resourceId] = pair<StaticNetwork*, SumoAdapterStatic>(network, adapter);
    return res;
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork)

void CreateBPRNetwork::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<bool>(getSuccess());
}

bool CreateBPRNetwork::Response::deserializeContents(stringstream &ss) {
    bool s;
    ss >> utils::deserialize<bool>(s);
    setSuccess(s);
    return true;
}

void CreateBPRNetwork::Response::handle(ostream &os) {
    os << "Content-type: text/html\n\n";
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork::Response)
