#include "Log/ProgressLoggerTableOStream.hpp"

#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLoggerTableOStream::ProgressLoggerTableOStream(ostream &os_):
    os(os_) {}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const StartMessage &) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot start message if state is not NO_MESSAGE");

    s = MESSAGE;
    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const EndMessage &) {
    if(s == TEXT)
        *this << EndText();

    if(s != MESSAGE)
        throw logic_error("Cannot end message if state is not MESSAGE");

    os << endl;
    s          = NO_MESSAGE;
    firstField = true;

    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const Progress &progress) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    if(!firstField) {
        os << "\t";
    }
    firstField = false;

    os << progress.p;

    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const Elapsed &elapsed) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send Progress if state is not MESSAGE");

    if(!firstField) {
        os << "\t";
    }
    firstField = false;

    os << elapsed.t;

    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const ETA &eta) {
    if(s == NO_MESSAGE)
        *this << StartMessage();

    if(s != MESSAGE)
        throw logic_error("Cannot send ETA if state is not MESSAGE");

    if(!firstField) {
        os << "\t";
    }
    firstField = false;

    os << eta.t;

    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const StartText &) {
    if(s == NO_MESSAGE)
        *this << StartMessage();
    if(s != MESSAGE)
        throw logic_error("Cannot start text if state is not MESSAGE");

    if(!firstField) {
        os << "\t";
    }
    firstField = false;

    s = TEXT;

    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const EndText &) {
    if(s != TEXT)
        throw logic_error("Cannot end text if state is not TEXT");
    s = MESSAGE;
    return *this;
}

ProgressLogger &ProgressLoggerTableOStream::operator<<(const int &t) { return send(t); }
ProgressLogger &ProgressLoggerTableOStream::operator<<(const double &t) { return send(t); }
ProgressLogger &ProgressLoggerTableOStream::operator<<(const char *t) { return send(t); }
