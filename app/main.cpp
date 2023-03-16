#include <iostream>

#include "GlobalState.hpp"
#include "data/SumoNetwork.hpp"
#include "network/CreateBPRNetwork.hpp"
#include "network/CreateStaticDemand.hpp"
#include "network/RunFWSimulation.hpp"
#include "network/Message.hpp"
#include "network/Socket.hpp"
#include "data/OFormatDemand.hpp"
#include "static/supply/BPRNetwork.hpp"
#include "data/SumoTAZs.hpp"
#include "static/StaticDemand.hpp"
#include "static/algos/FrankWolfe.hpp"
#include "static/algos/AllOrNothing.hpp"

using namespace std;

[[noreturn]] void loop();

int main() {
    // Setup
    MESSAGE_REGISTER_MAIN(CreateBPRNetwork);
    MESSAGE_REGISTER_MAIN(CreateBPRNetwork::Response);
    MESSAGE_REGISTER_MAIN(CreateStaticDemand);
    MESSAGE_REGISTER_MAIN(CreateStaticDemand::Response);
    MESSAGE_REGISTER_MAIN(RunFWSimulation);
    MESSAGE_REGISTER_MAIN(RunFWSimulation::Response);

    loop();

    return 0;
}

void loop() {
    Socket socket;
    socket.bind(8001);

    while (true) {
        Socket requestSocket = socket.accept();
        Message *m = requestSocket.receive();
        if (m->getType() == Message::Type::REQUEST) {
            MessageRequest *req = static_cast<MessageRequest *>(m);
            cerr << "Got request: " << req->getOperation() << endl;
            MessageResponse *res = req->process();
            requestSocket.send(res);
            delete res;
        } else {
            cerr << "App can only accept requests" << endl;
        }
        requestSocket.close();
        delete m;
    }
}
