#include "network/RequestCreateStaticNetwork.hpp"

using namespace std;

void RequestCreateStaticNetwork::serializeContents(stringstream &ss) const {
    // TODO
}

bool RequestCreateStaticNetwork::deserializeContents(stringstream &ss){
    // TODO
    return true;
}

MESSAGE_REGISTER_DEF(RequestCreateStaticNetwork)
