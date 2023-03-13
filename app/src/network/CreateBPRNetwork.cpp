#include "network/CreateBPRNetwork.hpp"

#include "data/SumoNetwork.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "utils/serialize.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

CreateBPRNetwork::CreateBPRNetwork() {}

CreateBPRNetwork::CreateBPRNetwork(const ResourceId &resourceId_, const string &path_)
    : resourceId(resourceId_), path(path_) {}

void CreateBPRNetwork::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(resourceId)
        << utils::serialize<string>(path);
}

bool CreateBPRNetwork::deserializeContents(stringstream &ss) {
    ss >> utils::deserialize<string>(resourceId) >> utils::deserialize<string>(path);
    return (bool)ss;
}

CreateBPRNetwork::Response *CreateBPRNetwork::process() {
    // SumoNetwork sumoNetwork;
    // sumoNetwork.loadFromFile(path);
    // StaticNetwork *network = BPRNetwork::fromSumoNetwork(sumoNetwork);
    // CreateBPRNetwork::Response *res = new CreateBPRNetwork::Response();
    // if(GlobalState::staticNetworks.count(resourceId)){
    //     res->setSuccess(false);
    //     return res;
    // }
    // GlobalState::staticNetworks[resourceId] = network;
    // return res;

    // TODO
    return nullptr;
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
