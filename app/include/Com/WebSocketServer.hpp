#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace Com {
class WebSocketServer {
    websocketpp::server<websocketpp::config::asio> srv;

    void wsHandler(websocketpp::connection_hdl hdl);
    void wsHandlerThread(websocketpp::connection_hdl hdl);
    void wsStringStream(websocketpp::connection_hdl hdl);

   public:
    WebSocketServer(int port = 9001);
    void loop();
};
}  // namespace Com
