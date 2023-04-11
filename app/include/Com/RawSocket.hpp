/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.h
 */

#pragma once

// #include "../Message/Message.h"
// #include "../Message/MessageFactory.h"

#include <netdb.h>

#include <string>

namespace Com {
class RawSocket {
   private:
    static const int        BACKLOG                = 10;
    static const int        NUMBER_RETRIES_CONNECT = 20000;
    static const useconds_t SLEEP_MICROS           = 200000;

    void init(const char *name, int port, bool is_listening);

    addrinfo *req = nullptr;
    int       sd;

   public:
    RawSocket();
    RawSocket(int sd);

    int getSd() const;

    void bind(int port);
    void connect(const std::string &name, int port);

    RawSocket accept();
    void      close();

    void  send(const char *msg, size_t sz);
    char *receive(size_t &sz);

    ~RawSocket();
};
}  // namespace Com
