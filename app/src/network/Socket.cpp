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
    string msg = m->serialize();
    socket.send(msg);
}

Message* Socket::receive() {
    string msg = socket.receive();

    stringstream ss(msg);

    return messageFactory.factoryMethod(ss);
}
