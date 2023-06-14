#include "Com/WebSocketServer.hpp"

#include <websocketpp/close.hpp>

#include "GlobalState.hpp"
#include "utils/string.hpp"

using namespace std;
using namespace Com;

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr                            message_ptr;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;

typedef websocketpp::log::alevel alevel;

WebSocketServer::WebSocketServer(int port) {
    // From https://github.com/zaphoyd/websocketpp/blob/master/examples/echo_server/echo_server.cpp
    try {
        cerr << "Starting WebSockets server" << endl;

        // Set logging settings
        srv.set_access_channels(alevel::all);
        srv.clear_access_channels(alevel::frame_header | alevel::frame_payload);

        // Initialize Asio
        srv.init_asio();

        // Register our message handler
        srv.set_open_handler(bind(&WebSocketServer::wsHandler, this, ::_1));

        // Listen on port 9001
        srv.listen((uint16_t)port);

        // Start the server accept loop
        srv.start_accept();

        cerr << "Started WebSockets server" << endl;
    } catch(const exception &e) {
        cerr << "Exception, what(): " << e.what() << endl;
    }
}

void WebSocketServer::loop() {
    // Start the ASIO io_service run loop
    srv.run();
}

void WebSocketServer::wsHandler(websocketpp::connection_hdl hdl) {
    thread t(&WebSocketServer::wsHandlerThread, this, hdl);
    t.detach();
}

void WebSocketServer::wsHandlerThread(websocketpp::connection_hdl hdl) {
    try {
        wsStringStream(hdl);
    } catch(const exception &e) {
        cerr << "wsHandlerThread: Exception, what(): " << e.what() << endl;
    }
}

void WebSocketServer::wsStringStream(websocketpp::connection_hdl hdl) {
    // From https://stackoverflow.com/questions/30514362/handle-websocketpp-connection-path
    server::connection_ptr con = srv.get_con_from_hdl(hdl);

    const string postfix = "/log";

    const string &resource = con->get_resource();
    assert(utils::ends_with(resource, postfix));
    const string resourceID = resource.substr(0, resource.size() - postfix.size());
    const string streamID   = "stream://" + resourceID;

    try {
        utils::pipestream &ios = GlobalState::streams.get(streamID);
        istream           &is  = ios.i();

        string payload;
        while(getline(is, payload)) {
            srv.send(hdl, payload, websocketpp::frame::opcode::binary);
        }
    } catch(const GlobalState::ResourceException &e) {
        srv.close(hdl, websocketpp::close::status::internal_endpoint_error, e.what());
        return;
    } catch(const iostream::failure &e) {
        cerr << "wsStringStream: Exception reading pipestream, what(): " << e.what()
             << endl;
        srv.close(hdl, websocketpp::close::status::internal_endpoint_error, "Exception, what(): "s + e.what());
    }

    srv.close(hdl, websocketpp::close::status::normal, "EOF");
}
