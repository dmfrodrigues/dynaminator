#include "Log/ProgressLogger.hpp"

#include <stdexcept>

using namespace Log;

ProgressLogger::ETA::ETA(double eta):
    t(eta) {}

ProgressLogger::Elapsed::Elapsed(double elapsed):
    t(elapsed) {}

ProgressLogger::Progress::Progress(double progress):
    p(progress) {}

ProgressLogger &ProgressLogger::operator<<(ProgressLogger& (*pf) (ProgressLogger&)){
    return pf(*this);
}
