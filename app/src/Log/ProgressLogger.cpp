#include "Log/ProgressLogger.hpp"

#include <stdexcept>

using namespace std;
using namespace Log;

ProgressLogger::ETA::ETA(double eta):
    t(eta) {}

ProgressLogger::Progress::Progress(double progress):
    p(progress) {}
