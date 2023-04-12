#pragma once

#include "Log/ProgressLogger.hpp"

namespace Log {
class ProgressLoggerJsonOStream: public ProgressLogger {
    template<class T>
    ProgressLogger &send(const T &t) {
        if(s != TEXT)
            throw std::logic_error("Cannot send if state is not TEXT");
        os << t;
        return *this;
    }

   public:
    std::ostream &os;

    bool firstField = true;

    enum State {
        NO_MESSAGE,
        MESSAGE,
        TEXT
    };
    State s = NO_MESSAGE;

   public:
    ProgressLoggerJsonOStream(std::ostream &os = std::cout);

    virtual ProgressLogger &operator<<(const ETA &eta);
    virtual ProgressLogger &operator<<(const Progress &progress);
    virtual ProgressLogger &operator<<(const StartText &);
    virtual ProgressLogger &operator<<(const EndText &);
    virtual ProgressLogger &operator<<(const StartMessage &);
    virtual ProgressLogger &operator<<(const EndMessage &);

    virtual ProgressLogger &operator<<(const int &t);
    virtual ProgressLogger &operator<<(const double &t);
    virtual ProgressLogger &operator<<(const char *t);
};

}
