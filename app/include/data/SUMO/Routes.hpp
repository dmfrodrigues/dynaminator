#pragma once

#include <memory>

#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"
#include "data/SUMO/TAZ.hpp"

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
}  // namespace SUMO
