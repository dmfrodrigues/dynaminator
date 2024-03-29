#include "Dynamic/Env/Event/EventLog.hpp"

#include <chrono>
#include <iomanip>

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

using clk = chrono::steady_clock;

EventLog::EventLog(
    Time                 t_,
    Time                 tStartSim_,
    Time                 tEndSim_,
    clk::time_point      tStart_,
    Log::ProgressLogger &logger_,
    Policy::Logger      &policyLogger_
):
    Event(t_),
    tStartSim(tStartSim_),
    tEndSim(tEndSim_),
    tStart(tStart_),
    logger(logger_),
    policyLogger(policyLogger_) {}

void EventLog::process(Env &env) {
    static Time prevTime = 0.0;

    static size_t prevNumberProcessedEvents = 0;

    static size_t nLEFT = 0, nLEFTGOOD = 0;

    const clk::time_point now = clk::now();

    double elapsed = (double)chrono::duration_cast<chrono::nanoseconds>(now - tStart).count() * 1e-9;

    double progress = (env.getTime() - tStartSim) / (tEndSim - tStartSim);

    double eta = (progress <= 0.0 ? 1.0 : elapsed * (1.0 - progress) / progress);

    static Time totalTime = 0.0;

    Time totalTimeInterval = 0.0;

    size_t nSTOPPED = 0, nMOVING = 0;

    size_t nLEFTInterval = 0, nLEFTGOODInterval = 0;
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
                if(vehicle.path.back().first > prevTime) {
                    nLEFT++;
                    nLEFTInterval++;
                    if(vehicle.toTAZ.sinks.count(vehicle.path.back().second.get().edge)) {
                        nLEFTGOOD++;
                        nLEFTGOODInterval++;
                        totalTimeInterval += Dt;
                        totalTime += Dt;
                    }
                }

                env.discardVehicle(vehicle);

                break;
            default:
                break;
        }
    }

    logger << Log::ProgressLogger::Elapsed(elapsed)
           << Log::ProgressLogger::Progress(progress)
           << Log::ProgressLogger::ETA(eta)
           << Log::ProgressLogger::StartText()
           << setprecision(1)
           << env.getTime()
           << setprecision(6)
           << "\t" << nSTOPPED + nMOVING + nLEFT
           << "\t" << nSTOPPED + nMOVING
           << "\t" << env.numberOfDespawnedVehicles()
           << "\t" << (nLEFTGOOD > 0 ? totalTime / (Time)nLEFTGOOD : 0)
           << "\t" << (nLEFTGOODInterval > 0 ? totalTimeInterval / (Time)nLEFTGOODInterval : 0)
           << "\t" << env.getNumberProcessedEvents() - prevNumberProcessedEvents
           << "\t";
    policyLogger.log(logger);
    logger << Log::ProgressLogger::EndMessage();

    prevTime                  = env.getTime();
    prevNumberProcessedEvents = env.getNumberProcessedEvents();
}
