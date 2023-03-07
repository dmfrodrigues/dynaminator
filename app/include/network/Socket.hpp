/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.h
 */

#pragma once

#include "network/RawSocket.hpp"

#include "network/Message.hpp"
#include "network/MessageFactory.hpp"

#include <string>
#include <netdb.h>

class Socket {
private:
    static MessageFactory messageFactory;

    RawSocket socket;

    Socket(RawSocket socket);
public:
    Socket();
    Socket(int sd);

    int getSd() const;

    void bind(int port);
    void connect(const std::string &name, int port);

    Socket accept();
    void close();

    void send(const Message *m);
    Message* receive();
};
