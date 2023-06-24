#include "Dynamic/Env/Event/EventLog.hpp"

#include <iomanip>

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

void EventLog::process(Env &env) {
    static Time prevTime = 0.0;

    const hrc::time_point now = hrc::now();

    double elapsed = (double)chrono::duration_cast<chrono::nanoseconds>(now - tStart).count() * 1e-9;

    double progress = (env.getTime() - tStartSim) / (tEndSim - tStartSim);

    double eta = (progress <= 0.0 ? 1.0 : elapsed * (1.0 - progress) / progress);

    Time totalTime = 0.0, totalTimeInterval = 0.0;

    size_t nSTOPPED = 0, nMOVING = 0, nLEFT = 0, nLEFTInterval = 0;
    for(const Vehicle &vehicle: env.getVehicles()) {
        Time Dt = vehicle.path.back().first - vehicle.path.front().first;

        switch(vehicle.state) {
            case Vehicle::State::STOPPED:
                nSTOPPED++;
                break;
            case Vehicle::State::MOVING:
                nMOVING++;
                break;
            case Vehicle::State::LEFT:
                nLEFT++;
                totalTime += Dt;
                if(vehicle.path.back().first > prevTime) {
                    nLEFTInterval++;
                    totalTimeInterval += Dt;
                }
                break;
            default:
                break;
        }
    }

    const size_t leave = env.getLeaveGood() + env.getLeaveBad();

    logger << Log::ProgressLogger::Elapsed(elapsed)
           << Log::ProgressLogger::Progress(progress)
           << Log::ProgressLogger::ETA(eta)
           << Log::ProgressLogger::StartText()
           << setprecision(1)
           << env.getTime()
           << setprecision(6)
           << "\t" << env.getNumberVehicles()
           << "\t" << nSTOPPED + nMOVING
           << "\t" << (leave > 0 ? (double)env.getLeaveGood() / leave : 0)
           << "\t" << (nLEFT > 0 ? totalTime / nLEFT : 0)
           << "\t" << (nLEFTInterval > 0 ? totalTimeInterval / nLEFTInterval : 0)
           << Log::ProgressLogger::EndMessage();

    prevTime = env.getTime();
}
