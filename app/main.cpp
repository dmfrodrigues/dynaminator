#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <websocketpp/close.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "Com/Message.hpp"
#include "Com/RunFWSimulation.hpp"
#include "Com/Socket.hpp"
#include "GlobalState.hpp"
#include "Static/Demand.hpp"
#include "Static/algos/AllOrNothing.hpp"
#include "Static/algos/FrankWolfe.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/TAZ.hpp"
#include "data/VISUM/OFormatDemand.hpp"

using namespace std;

[[noreturn]] void loop();

void loopWS();

int main() {
    // Setup
    MESSAGE_REGISTER_MAIN(Com::RunFWSimulation);

    thread websocketServerThread(loopWS);

    loop();

    return 0;
}

void loop() {
    cerr << "Starting server" << endl;

    Com::Socket socket;
    socket.bind(8001);

    cerr << "Started server" << endl;

    while(true) {
        Com::Socket   requestSocket = socket.accept();
        Com::Message *m             = requestSocket.receive();
        if(m->getType() == Com::Message::Type::REQUEST) {
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

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr                            message_ptr;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

void wsStringStream(server* s, websocketpp::connection_hdl hdl){
    // From https://stackoverflow.com/questions/30514362/handle-websocketpp-connection-path
    server::connection_ptr con = s->get_con_from_hdl(hdl);
    string resource = con->get_resource();

    cerr << "Connecting to resource " << resource << endl;

    shared_ptr<stringstream> ss;
    
    {
        lock_guard<mutex> lock(GlobalState::streams.mutex());

        try {
            ss = GlobalState::streams->at(resource);
        } catch(const out_of_range &e){
            s->close(hdl, websocketpp::close::status::internal_endpoint_error, "No such stream " + resource);
        }
    }

    while(!ss->eof()){
        string payload;
        getline(*ss, payload);
        s->send(hdl, payload, websocketpp::frame::opcode::binary);
    }

    // for(int i = 0; i <= 10; ++i){
    //     string payload = "Hello! i=" + to_string(i);
    //     try {
    //         s->send(hdl, payload, websocketpp::frame::opcode::binary);
    //         s->interrupt(hdl);
    //     } catch (websocketpp::exception const & e) {
    //         std::cout << "Echo failed because: "
    //                 << "(" << e.what() << ")" << std::endl;
    //     }
    //     if(i != 10)
    //         sleep(1);
    // }

    s->close(hdl, websocketpp::close::status::normal, "EOF");
}

void wsHandler(server* s, websocketpp::connection_hdl hdl) {
    cout << "wsHandler called" << endl;
    thread t(wsStringStream, s, hdl);
    t.detach();
}

void loopWS() {
    cerr << "Starting WebSockets server" << endl;

    // From https://github.com/zaphoyd/websocketpp/blob/master/examples/echo_server/echo_server.cpp

    // Create a server endpoint
    server echo_server;

    try {
        // Set logging settings
        echo_server.set_access_channels(websocketpp::log::alevel::all);
        echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        echo_server.init_asio();

        // Register our message handler
        echo_server.set_open_handler(bind(&wsHandler, &echo_server, ::_1));

        // Listen on port 9001
        echo_server.listen(9001);

        // Start the server accept loop
        echo_server.start_accept();

        cerr << "Started WebSockets server" << endl;

        // Start the ASIO io_service run loop
        echo_server.run();
    } catch(const exception &e){
        cerr << "Exception, what(): " << e.what() << endl;
    }
}
