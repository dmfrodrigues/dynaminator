#pragma once

#include <random>

#include "Dynamic/Policy/QLearner.hpp"

namespace Dynamic {
class DoubleQLearner: public QLearner {
   public:
    class PolicyFactory: public QLearner::Policy::Factory {
       public:
        PolicyFactory(
            Env::Env                                               &env,
            const SUMO::NetworkTAZs                                &sumo,
            const Dynamic::SUMOAdapter                             &adapter,
            std::random_device::result_type                         seed         = 0,
            std::optional<std::reference_wrapper<QLearner::Logger>> policyLogger = std::nullopt
        );

        virtual std::shared_ptr<Dynamic::Policy> create(
            Vehicle::ID     id,
            Time            depart,
            const Env::TAZ &fromTAZ,
            const Env::TAZ &toTAZ
        ) override;
    };

   private:
    QMatrixType QMatrixA, QMatrixB;

    std::mt19937 &gen;

   protected:
    virtual Reward estimateOptimalValue(const State &s) const override;
    virtual Reward estimateOptimalFutureValue(const State &s, const Action &a) const override;

    virtual Reward &Q(const State &s, const Action &a) override;
    virtual Reward  Q(const State &s, const Action &a) const override;

    virtual void updateMatrix(const State &s, const Action &a, Reward reward) override;

    Reward &QA(const State &s, const Action &a);
    Reward  QA(const State &s, const Action &a) const;

    Reward &QB(const State &s, const Action &a);
    Reward  QB(const State &s, const Action &a) const;

    Reward estimateOptimalValueA(const State &s) const;
    Reward estimateOptimalValueB(const State &s) const;

    Reward estimateOptimalFutureValueA(const State &s, const Action &a) const;
    Reward estimateOptimalFutureValueB(const State &s, const Action &a) const;

   public:
    DoubleQLearner(
        Env::Env                                               &env,
        const SUMO::Network                                    &network,
        const Dynamic::SUMOAdapter                             &adapter,
        const Env::TAZ                                         &destinationTAZ,
        std::mt19937                                           &gen,
        std::optional<std::reference_wrapper<QLearner::Logger>> policyLogger = std::nullopt,
        Reward                                                  alpha        = 0.5,
        Reward                                                  gamma        = 1.0,
        Reward                                                  xi           = 0.0,
        Reward                                                  eta          = 1.0,
        float                                                   epsilon      = 1.0e-3
    );
};
}  // namespace Dynamic
