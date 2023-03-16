/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.cpp
 */

#include "network/Socket.hpp"

#include <iostream>
#include <netdb.h>
#include <sys/un.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

MessageFactory Socket::messageFactory = MessageFactory();

Socket::Socket() : socket() {}

Socket::Socket(int sd) : socket(sd) { }

int Socket::getSd() const { return socket.getSd(); }

void Socket::bind(int port) {
   socket.bind(port);
}

void Socket::close() {
   socket.close();
}

Socket Socket::accept() {
    Socket ret;
    ret.socket = socket.accept();
    return ret;
}

void Socket::connect(const string &name, int port) {
   socket.connect(name, port);
}

void Socket::send(const Message *m) {
    stringstream ss = m->serialize();

    ss.seekg(0, ios::end);
    size_t sz = ss.tellg();

    socket.send(ss.str().data(), sz);
}

Message* Socket::receive() {
    size_t sz;
    char *buf = socket.receive(sz);

    stringstream ss;
    ss.write(buf, sz);
    delete buf;

    return messageFactory.factoryMethod(ss);
}
