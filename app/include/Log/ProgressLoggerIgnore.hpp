#pragma once

#include "Log/ProgressLogger.hpp"

namespace Log {
class ProgressLoggerIgnore: public ProgressLogger {
   public:
    static ProgressLoggerIgnore INSTANCE;

    virtual ProgressLoggerIgnore &operator<<(const Progress &progress);
    virtual ProgressLoggerIgnore &operator<<(const Elapsed &elapsed);
    virtual ProgressLoggerIgnore &operator<<(const ETA &eta);
    virtual ProgressLoggerIgnore &operator<<(const StartText &);
    virtual ProgressLoggerIgnore &operator<<(const EndText &);
    virtual ProgressLoggerIgnore &operator<<(const StartMessage &);
    virtual ProgressLoggerIgnore &operator<<(const EndMessage &);

    virtual ProgressLoggerIgnore &operator<<(const int &t);
    virtual ProgressLoggerIgnore &operator<<(const unsigned long &t);
    virtual ProgressLoggerIgnore &operator<<(const double &t);
    virtual ProgressLoggerIgnore &operator<<(const char *t);

    virtual ProgressLoggerIgnore &operator<<(std::_Setprecision f);

    ProgressLogger &operator<<(ProgressLogger &(*pf)(ProgressLogger &));

   protected:
    virtual ProgressLoggerIgnore &fixed();
};

}  // namespace Log
