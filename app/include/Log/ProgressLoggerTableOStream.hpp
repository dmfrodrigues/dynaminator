#pragma once

#include "Log/ProgressLogger.hpp"

namespace Log {
class ProgressLoggerTableOStream: public ProgressLogger {
    template<class T>
    ProgressLogger &send(const T &t) {
        if(s != TEXT)
            throw std::logic_error("Cannot send if state is not TEXT");
        os << t;
        return *this;
    }

   private:
    std::ostream &os;

    bool firstField = true;

    enum State {
        NO_MESSAGE,
        MESSAGE,
        TEXT
    };
    State s = NO_MESSAGE;

   public:
    ProgressLoggerTableOStream(std::ostream &os = std::cout);

    virtual ProgressLoggerTableOStream &operator<<(const Progress &progress);
    virtual ProgressLoggerTableOStream &operator<<(const Elapsed &elapsed);
    virtual ProgressLoggerTableOStream &operator<<(const ETA &eta);
    virtual ProgressLoggerTableOStream &operator<<(const StartText &);
    virtual ProgressLoggerTableOStream &operator<<(const EndText &);
    virtual ProgressLoggerTableOStream &operator<<(const StartMessage &);
    virtual ProgressLoggerTableOStream &operator<<(const EndMessage &);

    virtual ProgressLoggerTableOStream &operator<<(const int &t);
    virtual ProgressLoggerTableOStream &operator<<(const unsigned long &t);
    virtual ProgressLoggerTableOStream &operator<<(const double &t);
    virtual ProgressLoggerTableOStream &operator<<(const char *t);

    virtual ProgressLoggerTableOStream &operator<<(std::_Setprecision f);

    ProgressLogger &operator<<(ProgressLogger &(*pf)(ProgressLogger &));

   protected:
    virtual ProgressLoggerTableOStream &fixed();
};

}
