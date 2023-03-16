#include "network/CreateStaticDemand.hpp"

#include "data/OFormatDemand.hpp"
#include "static/StaticDemand.hpp"

using namespace std;

using ResourceId = GlobalState::ResourceId;

CreateStaticDemand::CreateStaticDemand() {}

CreateStaticDemand::CreateStaticDemand(
    const ResourceId &resourceId_,
    const ResourceId &networkId_,
    const string &path_
) :
    resourceId(resourceId_),
    networkId(networkId_),
    path(path_)
{}

void CreateStaticDemand::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(resourceId)
        << utils::serialize<string>(networkId)
        << utils::serialize<string>(path);
}

bool CreateStaticDemand::deserializeContents(stringstream &ss) {
    ss
        >> utils::deserialize<string>(resourceId)
        >> utils::deserialize<string>(networkId)
        >> utils::deserialize<string>(path);
    return (bool)ss;
}

CreateStaticDemand::Response *CreateStaticDemand::process() {
    const SumoAdapterStatic &adapter = GlobalState::staticNetworks.at(networkId).second;

    OFormatDemand oDemand = OFormatDemand::loadFromFile(path);
    StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

    CreateStaticDemand::Response *res = new CreateStaticDemand::Response();
    if (GlobalState::staticDemands.count(resourceId)) {
        res->setSuccess(false);
        return res;
    }
    GlobalState::staticDemands[resourceId] = demand;
    return res;
}

MESSAGE_REGISTER_DEF(CreateStaticDemand)

void CreateStaticDemand::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<bool>(getSuccess());
}

bool CreateStaticDemand::Response::deserializeContents(stringstream &ss) {
    bool s;
    ss >> utils::deserialize<bool>(s);
    setSuccess(s);
    return true;
}

void CreateStaticDemand::Response::handle(ostream &os) {
    os << "Content-type: application/json\n\n";
}

MESSAGE_REGISTER_DEF(CreateStaticDemand::Response)
