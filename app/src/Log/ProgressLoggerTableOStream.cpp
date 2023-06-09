#include "Log/ProgressLoggerTableOStream.hpp"

#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLoggerTableOStream::ProgressLoggerTableOStream(ostream &os_):
    os(os_) {}

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const StartMessage &) {
    if(s != NO_MESSAGE)
        throw logic_error("Cannot start message if state is not NO_MESSAGE");

    s = MESSAGE;
    return *this;
}

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const EndMessage &) {
    if(s == TEXT)
        *this << EndText();

    if(s != MESSAGE)
        throw logic_error("Cannot end message if state is not MESSAGE");

    os << endl;
    s          = NO_MESSAGE;
    firstField = true;

    return *this;
}

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const Progress &progress) {
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

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const Elapsed &elapsed) {
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

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const ETA &eta) {
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

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const StartText &) {
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

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const EndText &) {
    if(s != TEXT)
        throw logic_error("Cannot end text if state is not TEXT");
    s = MESSAGE;
    return *this;
}

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const int &t) { send(t); return *this; }
ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const unsigned long &t) { send(t); return *this; }
ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const double &t) { send(t); return *this; }
ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(const char *t) { send(t); return *this; }

ProgressLoggerTableOStream &ProgressLoggerTableOStream::operator<<(std::_Setprecision t) { os << t; return *this; }

ProgressLogger &ProgressLoggerTableOStream::operator<<(ProgressLogger& (*pf) (ProgressLogger&)){
    return pf(*this);
}

ProgressLoggerTableOStream &ProgressLoggerTableOStream::fixed() { os << std::fixed; return *this; }
