#include <iostream>

#include "network/Message.hpp"
#include "network/RequestCreateStaticNetwork.hpp"
#include "network/SumoNetwork.hpp"

using namespace std;

int main() {
    // Setup
    MESSAGE_REGISTER_MAIN(RequestCreateStaticNetwork);

    SumoNetwork network = SumoNetwork::loadFromFile("data/network/net.net.xml");

    return 0;
}
