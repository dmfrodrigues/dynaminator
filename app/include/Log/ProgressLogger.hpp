#pragma once

#include <exception>
#include <iostream>

namespace Log {
class ProgressLogger {
   public:
    struct ETA {
        double t;

        ETA(double eta);
    };

    struct Progress {
        double p;

        Progress(double progress);
    };

    struct StartMessage {};
    struct EndMessage {};

    struct StartText {};
    struct EndText {};

    virtual ProgressLogger &operator<<(const ETA &eta)           = 0;
    virtual ProgressLogger &operator<<(const Progress &progress) = 0;
    virtual ProgressLogger &operator<<(const StartText &)        = 0;
    virtual ProgressLogger &operator<<(const EndText &)          = 0;
    virtual ProgressLogger &operator<<(const StartMessage &)     = 0;
    virtual ProgressLogger &operator<<(const EndMessage &)       = 0;

    virtual ProgressLogger &operator<<(const int &t)    = 0;
    virtual ProgressLogger &operator<<(const double &t) = 0;
    virtual ProgressLogger &operator<<(const char *t)   = 0;
};

}  // namespace Log
