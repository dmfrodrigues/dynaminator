#include "Log/ProgressLoggerIgnore.hpp"

#include <stdexcept>

using namespace std;
using namespace Log;

Log::ProgressLoggerIgnore Log::ProgressLoggerIgnore::INSTANCE;

// clang-format off
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const StartMessage   &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const EndMessage     &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const Progress       &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const Elapsed        &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const ETA            &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const StartText      &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const EndText        &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const int            &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const unsigned long  &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const double         &) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(const char           *) { return *this; }
ProgressLoggerIgnore &ProgressLoggerIgnore::operator<<(std::_Setprecision    ) { return *this; }
// clang-format on

ProgressLogger &ProgressLoggerIgnore::operator<<(ProgressLogger &(*)(ProgressLogger &)) { return *this; }

ProgressLoggerIgnore &ProgressLoggerIgnore::fixed() { return *this; }
