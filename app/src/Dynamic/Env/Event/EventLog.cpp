#include "Dynamic/Env/Event/EventLog.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

using hrc = chrono::high_resolution_clock;

EventLog::EventLog(Time t_, Time tStartSim_, Time tEndSim_, hrc::time_point tStart_, Log::ProgressLogger &logger_):
    Event(t_),
    tStartSim(tStartSim_),
    tEndSim(tEndSim_),
    tStart(tStart_),
    logger(logger_) {}

void EventLog::process(Env &env) const {
    const hrc::time_point now = hrc::now();

    double elapsed = (double)chrono::duration_cast<chrono::nanoseconds>(now - tStart).count() * 1e-9;

    double progress = (env.getTime() - tStartSim) / (tEndSim - tStartSim);

    double eta = (progress <= 0.0 ? 1.0 : elapsed * (1.0 - progress) / progress);

    logger << Log::ProgressLogger::Elapsed(elapsed)
           << Log::ProgressLogger::Progress(progress)
           << Log::ProgressLogger::ETA(eta)
           << Log::ProgressLogger::StartText()
           << env.getTime()
           << "\t" << env.getNumberVehicles()
           << "\t" << env.getQueueSize()
           << Log::ProgressLogger::EndMessage();
}

