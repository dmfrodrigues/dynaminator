#pragma once

#include <memory>

#include "Static/Solution.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "data/SUMO/TAZ.hpp"
#include "data/SumoAdapterStatic.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wconversion"
#include <color/color.hpp>
#pragma GCC diagnostic pop

namespace SUMO {
class Routes {
   public:
    template<typename T, typename... args>
    class Loader {
       public:
        Routes *load(T var1, args... var2);
    };

    class Flow {
        friend Routes;

       public:
        typedef SUMO::ID ID;
        enum class DepartPosEnum : int {
            RANDOM,
            FREE,
            RANDOM_FREE,
            BASE,
            LAST,
            STOP
        };
        union DepartPos {
            float         f;
            DepartPosEnum e;
        };

        enum class DepartSpeedEnum : int {
            RANDOM,
            MAX,
            DESIRED,
            SPEED_LIMIT,
            LAST,
            AVG
        };
        union DepartSpeed {
            float           f;
            DepartSpeedEnum e;
        };

        struct Policy {
        };
        struct PolicyVehsPerHour: public Policy {
            double vehsPerHour;
            PolicyVehsPerHour(double vehsPerHour);
        };
        struct PolicyPeriod: public Policy {
            union {
                float       f;
                std::string s;
            } period;
        };
        struct PolicyProbability: public Policy {
            double probability;
        };

       private:
        ID                               id;
        std::optional<color::rgb<float>> color;
        SUMO::Time                       begin;
        SUMO::Time                       end;
        std::optional<SUMO::TAZ::ID>     fromTaz;
        std::optional<SUMO::TAZ::ID>     toTaz;
        std::optional<DepartPos>         departPos;
        std::optional<DepartSpeed>       departSpeed;
        std::shared_ptr<Policy>          policy;

        Flow(ID id, SUMO::Time begin, SUMO::Time end, std::shared_ptr<Policy> policy);

       public:
        void setColor(color::rgb<float> color);
        void setFromTaz(SUMO::TAZ::ID fromTaz);
        void setToTaz(SUMO::TAZ::ID toTaz);
        void setDepartPos(DepartPos departPos);
        void setDepartSpeed(DepartSpeed departSpeed);
    };

   private:
    std::map<Flow::ID, Flow> flows;

   public:
    Flow &createFlow(SUMO::ID id, SUMO::Time begin, SUMO::Time end, std::shared_ptr<Flow::Policy> policy);
};

// clang-format off
template<>
class Routes::Loader<
    const Static::Network &,
    const Static::Solution &,
    const SumoAdapterStatic &
> {
   public:
    Routes *load(
        const Static::Network &network,
        const Static::Solution &x,
        const SumoAdapterStatic &adapter
    );
};
// clang-format on

}  // namespace SUMO
