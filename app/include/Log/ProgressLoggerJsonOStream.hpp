#pragma once

#include <nlohmann/json.hpp>

#include "Log/ProgressLogger.hpp"

namespace Log {
class ProgressLoggerJsonOStream: public ProgressLogger {
    template<class T>
    ProgressLogger &send(const T &t) {
        if(s != TEXT)
            throw std::logic_error("Cannot send if state is not TEXT");
        textStream << t;
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

    std::stringstream textStream;
    nlohmann::json    jsonMessage;

   public:
    ProgressLoggerJsonOStream(std::ostream &os = std::cout);

    virtual ProgressLoggerJsonOStream &operator<<(const Progress &progress);
    virtual ProgressLoggerJsonOStream &operator<<(const Elapsed &elapsed);
    virtual ProgressLoggerJsonOStream &operator<<(const ETA &eta);
    virtual ProgressLoggerJsonOStream &operator<<(const StartText &);
    virtual ProgressLoggerJsonOStream &operator<<(const EndText &);
    virtual ProgressLoggerJsonOStream &operator<<(const StartMessage &);
    virtual ProgressLoggerJsonOStream &operator<<(const EndMessage &);

    virtual ProgressLoggerJsonOStream &operator<<(const int &t);
    virtual ProgressLoggerJsonOStream &operator<<(const unsigned long &t);
    virtual ProgressLoggerJsonOStream &operator<<(const double &t);
    virtual ProgressLoggerJsonOStream &operator<<(const char *t);

    virtual ProgressLoggerJsonOStream &operator<<(std::_Setprecision f);

   protected:
    virtual ProgressLoggerJsonOStream &fixed();
};

}  // namespace Log
