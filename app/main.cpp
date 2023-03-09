#include <iostream>

#include "GlobalState.hpp"
#include "data/SumoNetwork.hpp"
#include "network/CreateBPRNetwork.hpp"
#include "network/Message.hpp"
#include "network/Socket.hpp"

using namespace std;

GlobalState global;

[[ noreturn ]] void loop();

int main() {
    // Setup
    MESSAGE_REGISTER_MAIN(CreateBPRNetwork);

    loop();

    return 0;
}

void loop(){
    Socket socket;
    socket.bind(8001);

    while(true){
        Socket requestSocket = socket.accept();
        Message *m = requestSocket.receive();
        if(m->getType() == Message::Type::REQUEST){
            MessageRequest *req = static_cast<MessageRequest*>(m);
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
