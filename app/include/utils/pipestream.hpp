#pragma once

#include <ext/stdio_filebuf.h>
#include <iostream>
#include <ostream>

namespace utils {
class pipestream {
    int ifs, ofs;
    std::istream is;
    std::ostream os;
    __gnu_cxx::stdio_filebuf<char> iBuf;
    __gnu_cxx::stdio_filebuf<char> oBuf;

    pipestream(std::pair<int, int> fds);

   public:
    pipestream();

    std::istream &i();
    std::ostream &o();

    void closeWrite();
};
}  // namespace utils
