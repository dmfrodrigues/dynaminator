#pragma once

#include "Log/ProgressLogger.hpp"
#include "Static/Demand.hpp"
#include "Static/Solution.hpp"
#include "Static/algos/FrankWolfe.hpp"

namespace Static {
template<typename NetworkType, typename FrankWolfeType>
class IterativeEquilibration {
    FrankWolfeType &fw;

    Log::ProgressLogger &logger;
    
    double epsilon;
    size_t iterations = 1000;

   public:
    IterativeEquilibration(
        FrankWolfeType &fw_,
        Log::ProgressLogger &logger_
    ) : fw(fw_), logger(logger_) {}

    void setIterations(size_t it) {
        iterations = it;
    }

    void setStopCriteria(double stopCriteria){
        epsilon = stopCriteria;
    }

    Solution solve(
        const NetworkType &network,
        const Demand      &demand,
        const Solution    &startingSolution
    ) {
        typedef std::chrono::high_resolution_clock hrc;

        logger << Log::ProgressLogger::Elapsed(0)
               << Log::ProgressLogger::Progress(0)
               << Log::ProgressLogger::StartText()
               << "it\tzn"
               << Log::ProgressLogger::EndMessage();
        
        const hrc::time_point tStart = hrc::now();

        Solution xn = startingSolution;

        double zn = network.evaluate(xn);

        Solution xBest = xn;
        double zBest = zn;

        logger << Log::ProgressLogger::Elapsed(0)
                << Log::ProgressLogger::Progress(0)
                << Log::ProgressLogger::StartText()
                << 0
                << "\t" << zn
                << Log::ProgressLogger::EndMessage();

        for (size_t it = 1; it <= iterations; it++) {
            Solution xPrev = xn;
            xn = fw.solve(
                network.makeConvex(xn),
                demand,
                xn
            );

            zn = network.evaluate(xn);

            const hrc::time_point t = hrc::now();

            double elapsed = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t - tStart).count() * 1e-9;

            double progress = (double)it / (double)iterations;

            logger << Log::ProgressLogger::Elapsed(elapsed)
                   << Log::ProgressLogger::Progress(progress)
                   << Log::ProgressLogger::StartText()
                   << it
                   << "\t" << zn
                   << Log::ProgressLogger::EndMessage();

            if(zn < zBest){
                xBest = xn;
                zBest = zn;
            }
        }
        return xBest;
    }
};
}  // namespace Static
