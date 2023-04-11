/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.h
 */

#pragma once

#include <netdb.h>

#include <string>

#include "Com/Message.hpp"
#include "Com/MessageFactory.hpp"
#include "Com/RawSocket.hpp"

namespace Com {
class Socket {
   private:
    static MessageFactory messageFactory;

    RawSocket socket;

    Socket(RawSocket socket);

   public:
    Socket();
    Socket(int sd);

    int getSd() const;

    void   bind(int port);
    void   connect(const std::string &name, int port);
    Socket accept();
    void   close();

    void     send(const Message *m);
    Message *receive();
};
}  // namespace Com
