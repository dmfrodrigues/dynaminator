#include "Log/ProgressLoggerJsonOStream.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#pragma GCC diagnostic ignored "-Wswitch-enum"
#pragma GCC diagnostic ignored "-Wswitch-default"
#include <httplib.h>
#pragma GCC diagnostic pop

#include <ostream>
#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLoggerJsonOStream::ProgressLoggerJsonOStream(ostream &os_):
    os(os_) {}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const StartMessage &) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot start message if state is not NO_MESSAGE");

    s = MESSAGE;
    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const EndMessage &) {
    if(s == TEXT)
        *this << EndText();

    if(s != MESSAGE)
        throw logic_error("Cannot end message if state is not MESSAGE");

    os << jsonMessage.dump() << endl;
    jsonMessage.clear();

    s          = NO_MESSAGE;
    firstField = true;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const Progress &progress) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    jsonMessage["progress"] = progress.p;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const Elapsed &elapsed) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    jsonMessage["elapsed"] = elapsed.t;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const ETA &eta) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send ETA if state is not MESSAGE");

    jsonMessage["eta"] = eta.t;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const StartText &) {
    if(s == NO_MESSAGE)
        *this << StartMessage();
    if(s != MESSAGE)
        throw logic_error("Cannot start text if state is not MESSAGE");

    s = TEXT;

    textStream.str("");
    textStream.clear();

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const EndText &) {
    if(s != TEXT)
        throw logic_error("Cannot end text if state is not TEXT");

    string text = textStream.str();

    jsonMessage["message"] = text;

    s = MESSAGE;

    return *this;
}

// clang-format off
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const int &t) { send(t); return *this; }
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const unsigned long &t) { send(t); return *this; }
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const double &t) { send(t); return *this; }
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const char *t) { send(t); return *this; }

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(std::_Setprecision t) { os << t; textStream << t; return *this; }

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::fixed() { os << std::fixed; textStream << std::fixed; return *this; }
// clang-format on
