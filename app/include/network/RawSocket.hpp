/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.h
 */

#pragma once

// #include "../Message/Message.h"
// #include "../Message/MessageFactory.h"

#include <string>
#include <netdb.h>

class RawSocket {
private:
    static const int BACKLOG = 10;
    static const int NUMBER_RETRIES_CONNECT = 20000;
    static const useconds_t SLEEP_MICROS = 200000;

    void init(const char *name, int port, bool is_listening);

    addrinfo *req = nullptr;
    int sd;

public:
    RawSocket();
    RawSocket(int sd);

    int getSd() const;

    void bind(int port);
    void connect(const std::string &name, int port);

    RawSocket accept();
    void close();

    void send(const std::string &msg);
    std::string receive();

    ~RawSocket();
};
