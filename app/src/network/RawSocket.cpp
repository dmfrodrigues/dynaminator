/**
 * From https://github.com/dmfrodrigues/tum-cbdp-ex4/blob/master/Socket/Socket.cpp
 */

#include "network/RawSocket.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

using namespace std;

RawSocket::RawSocket() {}

RawSocket::RawSocket(int fd) : sd(fd) {}

int RawSocket::getSd() const {
    return sd;
}

void RawSocket::init(const char *name, int port, bool is_listening) {
    int ret;

    addrinfo hints{};
    memset(&hints, 0, sizeof(addrinfo));

    hints.ai_addrlen = sizeof(struct sockaddr_in);

    if (is_listening) {
        hints.ai_flags = AI_PASSIVE;
        hints.ai_family = AF_INET6;
        hints.ai_socktype = SOCK_STREAM;
    } else {
        hints.ai_flags = IPPROTO_TCP;
        hints.ai_socktype = SOCK_STREAM;
    }

    if ((ret = getaddrinfo(name, to_string(port).c_str(), &hints, &req)) != 0) {
        throw runtime_error("getaddrinfo() failed: " + string(gai_strerror(ret)));
    }

    sd = socket(req->ai_family, req->ai_socktype, req->ai_protocol);
    if (sd == -1) {
        throw runtime_error("socket() failed");
    }

    // allow kernel to rebind address even when in TIME_WAIT state
    int yes = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
        throw runtime_error("setsockopt() failed");
    }
}

void RawSocket::bind(int port) {
    init(NULL, port, true);

    if (::bind(sd, req->ai_addr, req->ai_addrlen) == -1) {
        throw runtime_error("perform_bind() failed");
    }

    if (listen(sd, BACKLOG) == -1) {
        throw runtime_error("perform_listen() failed");
    }
}

void RawSocket::close() {
    ::close(sd);
}

RawSocket RawSocket::accept() {
    int new_fd = ::accept(sd, nullptr, nullptr);
    return RawSocket(new_fd);
}

void RawSocket::connect(const string &name, int port) {
    init(name.c_str(), port, false);

    bool connected = false;
    int i = 0;
    while (!connected && i < NUMBER_RETRIES_CONNECT) {
        if (::connect(sd, req->ai_addr, req->ai_addrlen) == -1) {
            cerr << "[W] Failed to connect to " << name << " " << port << endl;
            usleep(SLEEP_MICROS);

            ++i;
        } else {
            connected = true;
        }
    }

    if (!connected)
        throw runtime_error("connect() failed");
}

void RawSocket::send(const char *msg, size_t sz) {
    if(
        write(sd, &sz, sizeof(sz)) != sizeof(sz) ||
        write(sd, msg, sz)  != (ssize_t)sz
    ) throw ios_base::failure("Failed to write entire message");
}

char* RawSocket::receive(size_t &sz) {
    size_t n = read(sd, &sz, sizeof(sz));
    if (n == 0) return nullptr;

    char *buf = new char[sz + 1];

    char *current_buf = buf;
    ssize_t left_to_read = sz;
    while (left_to_read > 0) {
        n = read(sd, current_buf, left_to_read);
        if (n == 0) return nullptr;
        current_buf += n;
        left_to_read -= n;
    }
    buf[sz] = '\0';

    return buf;
}

RawSocket::~RawSocket() {
    freeaddrinfo(req);
}
