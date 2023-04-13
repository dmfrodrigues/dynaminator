#include "utils/pipestream.hpp"

#include <unistd.h>

#include <stdexcept>

using namespace std;
using namespace utils;

pair<int, int> create_pipe() {
    int fds[2];
    if(pipe(fds)) {
        throw runtime_error("Failed to create pipe");
    }
    return pair<int, int>(fds[0], fds[1]);
}

pipestream::pipestream():
    pipestream(create_pipe()) {}

pipestream::pipestream(pair<int, int> fds):
    ifs(fds.first),
    ofs(fds.second),
    is(0),
    os(0),
    iBuf(fds.first, ios::in),
    oBuf(fds.second, ios::out) {
    is.rdbuf(&iBuf);
    os.rdbuf(&oBuf);

    is.exceptions(
        iostream::badbit
    );
    os.exceptions(
        iostream::failbit | iostream::badbit
    );
}

istream &pipestream::i() { return is; }
ostream &pipestream::o() { return os; }

void pipestream::closeWrite() {
    close(ofs);
}
