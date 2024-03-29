#pragma once

#include <exception>
#include <iomanip>
#include <iostream>

namespace Log {
class ProgressLogger;
}  // namespace Log

namespace std {
inline Log::ProgressLogger &fixed(Log::ProgressLogger &logger);
}  // namespace std

namespace Log {
class ProgressLogger {
   public:
    struct Progress {
        double p;

        Progress(double progress);
    };

    struct Elapsed {
        double t;

        Elapsed(double t);
    };

    struct ETA {
        double t;

        ETA(double eta);
    };

    struct StartMessage {};
    struct EndMessage {};

    struct StartText {};
    struct EndText {};

    virtual ProgressLogger &operator<<(const Progress &progress) = 0;
    virtual ProgressLogger &operator<<(const Elapsed &elapsed)   = 0;
    virtual ProgressLogger &operator<<(const ETA &eta)           = 0;
    virtual ProgressLogger &operator<<(const StartText &)        = 0;
    virtual ProgressLogger &operator<<(const EndText &)          = 0;
    virtual ProgressLogger &operator<<(const StartMessage &)     = 0;
    virtual ProgressLogger &operator<<(const EndMessage &)       = 0;

    virtual ProgressLogger &operator<<(const int &t)           = 0;
    virtual ProgressLogger &operator<<(const unsigned long &t) = 0;
    virtual ProgressLogger &operator<<(const double &t)        = 0;
    virtual ProgressLogger &operator<<(const char *t)          = 0;

    virtual ProgressLogger &operator<<(std::_Setprecision f) = 0;

    ProgressLogger &operator<<(ProgressLogger &(*pf)(ProgressLogger &));

   protected:
    friend ProgressLogger & ::std::fixed(ProgressLogger &logger);
    virtual ProgressLogger &fixed() = 0;
};

}  // namespace Log

namespace std {
inline Log::ProgressLogger &fixed(Log::ProgressLogger &logger) {
    return logger.fixed();
}
}  // namespace std
