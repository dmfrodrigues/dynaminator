#include "Log/ProgressLoggerJsonOStream.hpp"

#include <ostream>
#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLoggerJsonOStream::ProgressLoggerJsonOStream(ostream &os_):
    os(os_) {}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const StartMessage &) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot start message if state is not NO_MESSAGE");

    os << "{";
    s = MESSAGE;
    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const EndMessage &) {
    if(s == TEXT)
        *this << EndText();

    if(s != MESSAGE)
        throw logic_error("Cannot end message if state is not MESSAGE");

    os << "}" << endl;
    s          = NO_MESSAGE;
    firstField = true;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const Progress &progress) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    if(!firstField) {
        os << ",";
    }
    firstField = false;

    os << "\"progress\":" << progress.p;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const Elapsed &elapsed) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    if(!firstField) {
        os << ",";
    }
    firstField = false;
    os << "\"elapsed\":" << elapsed.t;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const ETA &eta) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send ETA if state is not MESSAGE");

    if(!firstField) {
        os << ",";
    }
    firstField = false;

    os << "\"eta\":" << eta.t;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const StartText &) {
    if(s == NO_MESSAGE)
        *this << StartMessage();
    if(s != MESSAGE)
        throw logic_error("Cannot start text if state is not MESSAGE");

    if(!firstField) {
        os << ",";
    }
    firstField = false;

    os << "\"message\":\"";
    s = TEXT;

    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const EndText &) {
    if(s != TEXT)
        throw logic_error("Cannot end text if state is not TEXT");
    os << "\"";
    s = MESSAGE;
    return *this;
}

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const int &t) { send(t); return *this; }
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const double &t) { send(t); return *this; }
ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(const char *t) { send(t); return *this; }

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::operator<<(std::_Setprecision t) { os << t; return *this; }

ProgressLoggerJsonOStream &ProgressLoggerJsonOStream::fixed() { os << std::fixed; return *this; }
