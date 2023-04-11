#include <iostream>

#include "GlobalState.hpp"
#include "data/SUMO/Network.hpp"
#include "Com/RunFWSimulation.hpp"
#include "Com/Message.hpp"
#include "Com/Socket.hpp"
#include "data/VISUM/OFormatDemand.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/TAZ.hpp"
#include "Static/Demand.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/algos/AllOrNothing.hpp"

using namespace std;

[[noreturn]] void loop();

int main() {
    // Setup
    MESSAGE_REGISTER_MAIN(Com::RunFWSimulation);

    loop();

    return 0;
}

void loop() {
    cerr << "Starting simulator" << endl;

    Com::Socket socket;
    socket.bind(8001);

    cerr << "Simulator started" << endl;

    while (true) {
        Com::Socket requestSocket = socket.accept();
        Com::Message *m = requestSocket.receive();
        if (m->getType() == Com::Message::Type::REQUEST) {
            Com::MessageRequest *req = static_cast<Com::MessageRequest *>(m);
            cerr << "Got request: " << req->getOperation() << endl;
            Com::MessageResponse *res = req->process();
            requestSocket.send(res);
            delete res;
        } else {
            cerr << "App can only accept requests" << endl;
        }
        requestSocket.close();
        delete m;
    }
}
