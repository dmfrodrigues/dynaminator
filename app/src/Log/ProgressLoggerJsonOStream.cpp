#include "Log/ProgressLoggerJsonOStream.hpp"

#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLoggerJsonOStream::ProgressLoggerJsonOStream(ostream &os_):
    os(os_) {}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const ETA &eta) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot send ETA before sending previous message");
    os << "{"
       << "\"eta\":" << eta.t
       << "}"
       << endl;
    return *this;
}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const StartMessage &) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot start message if state is not NO_MESSAGE");

    if(!firstField) {
        os << ",";
    }
    firstField = false;

    os << "{";
    s = MESSAGE;
    return *this;
}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const EndMessage &) {
    if(s == TEXT)
        *this << EndText();

    if(s != MESSAGE)
        throw logic_error("Cannot end message if state is not MESSAGE");

    os << "}" << endl;
    s          = NO_MESSAGE;
    firstField = true;

    return *this;
}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const Progress &progress) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    os << "\"progress\":" << progress.p;

    return *this;
}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const StartText &) {
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

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const EndText &) {
    if(s != TEXT)
        throw logic_error("Cannot end text if state is not TEXT");
    os << "\"";
    s = MESSAGE;
    return *this;
}

ProgressLogger &ProgressLoggerJsonOStream::operator<<(const int &t) { return send(t); }
ProgressLogger &ProgressLoggerJsonOStream::operator<<(const double &t) { return send(t); }
ProgressLogger &ProgressLoggerJsonOStream::operator<<(const char *t) { return send(t); }
