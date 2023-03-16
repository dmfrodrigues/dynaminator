#include "network/CreateStaticDemand.hpp"

#include "HttpStatusCodes_C++.h"
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
    CreateStaticDemand::Response *res = new CreateStaticDemand::Response();
    try {
        const SumoAdapterStatic &adapter = GlobalState::staticNetworks.at(networkId).second;

        OFormatDemand oDemand = OFormatDemand::loadFromFile(path);
        StaticDemand demand = StaticDemand::fromOFormat(oDemand, adapter);

        if (GlobalState::staticDemands.count(resourceId)) {
            res->setStatusCode(400);
            res->setReason("Static demand with id " + resourceId + " already exists");
            return res;
        }
        GlobalState::staticDemands[resourceId] = demand;
        return res;
    } catch(const exception &e){
        res->setStatusCode(500);
        res->setReason("what(): " + string(e.what()));
        return res;
    }
}

MESSAGE_REGISTER_DEF(CreateStaticDemand)

void CreateStaticDemand::Response::serializeContents(stringstream &ss) const {
    ss << utils::serialize<int>(getStatusCode()) << utils::serialize<string>(getReason());
}

bool CreateStaticDemand::Response::deserializeContents(stringstream &ss) {
    int s;
    ss >> utils::deserialize<int>(s);
    setStatusCode(s);
    string r;
    ss >> utils::deserialize<string>(r);
    setReason(r);
    return true;
}

void CreateStaticDemand::Response::handle(ostream &os) {
    if(getStatusCode() == 200)
        os << "Content-type: application/json\n\n";
    else
        os << "Status: " << getStatusCode() << " " << HttpStatus::reasonPhrase(getStatusCode()) << "\n";
}

MESSAGE_REGISTER_DEF(CreateStaticDemand::Response)
