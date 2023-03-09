#include "network/CreateBPRNetwork.hpp"

#include "utils/serialize.hpp"

using namespace std;

void CreateBPRNetwork::serializeContents(stringstream &ss) const {
    ss
        << utils::serialize<string>(resourceId)
        << utils::serialize<string>(path);
}

bool CreateBPRNetwork::deserializeContents(stringstream &ss){
    ss
        >> utils::deserialize<string>(resourceId)
        >> utils::deserialize<string>(path);
    return (bool)ss;
}

CreateBPRNetwork::Response* CreateBPRNetwork::process(){
    // TODO
    return nullptr;
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork)

void CreateBPRNetwork::Response::serializeContents(stringstream &ss) const {
    // TODO
}

bool CreateBPRNetwork::Response::deserializeContents(stringstream &ss){
    // TODO
    return true;
}

MESSAGE_REGISTER_DEF(CreateBPRNetwork::Response)
