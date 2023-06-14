#pragma once

#include <memory>

#include "Static/SUMOAdapter.hpp"
#include "Static/Solution.hpp"
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
    template<typename T, typename... args>
    class Loader {
       public:
        Routes load(T var1, args... var2);
    };

    class Flow {
        friend Routes;

       public:
        typedef SUMO::ID ID;
        struct DepartPos {
            enum class Enum {
                RANDOM,
                FREE,
                RANDOM_FREE,
                BASE,
                LAST,
                STOP
            };
            std::optional<Enum>  e;
            std::optional<float> f;
        };

        struct DepartSpeed {
            enum class Enum {
                RANDOM,
                MAX,
                DESIRED,
                SPEED_LIMIT,
                LAST,
                AVG
            };
            std::optional<Enum>  e;
            std::optional<float> f;
        };

        struct Policy {
            virtual void saveToXML(rapidxml::xml_node<> &flowEl) const = 0;
        };
        struct PolicyVehsPerHour: public Policy {
            double vehsPerHour;
            PolicyVehsPerHour(double vehsPerHour);
            virtual void saveToXML(rapidxml::xml_node<> &flowEl) const override;
        };
        struct PolicyPeriod: public Policy {
            std::optional<double>      f;
            std::optional<std::string> s;
            virtual void               saveToXML(rapidxml::xml_node<> &flowEl) const override;
        };
        struct PolicyProbability: public Policy {
            double       probability;
            virtual void saveToXML(rapidxml::xml_node<> &flowEl) const override;
        };

       private:
        ID                               id;
        std::optional<color::rgb<float>> color;
        Time                             begin;
        Time                             end;
        std::shared_ptr<Policy>          policy;
        Route                            route;
        std::optional<TAZ::ID>           fromTaz;
        std::optional<TAZ::ID>           toTaz;
        std::optional<DepartPos>         departPos;
        std::optional<DepartSpeed>       departSpeed;

        Flow(
            ID                      id,
            Time                    begin,
            Time                    end,
            std::shared_ptr<Policy> policy,
            Route                   route
        );

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
    Flow &createFlow(
        ID                            id,
        Time                          begin,
        Time                          end,
        std::shared_ptr<Flow::Policy> policy,
        Route                         route
    );

    void saveToFile(const std::string &filePath) const;
};

// clang-format off
template<>
class Routes::Loader<
    const Static::Network &,
    const Static::Solution &,
    const Static::SUMOAdapter &
> {
   public:
    Routes load(
        const Static::Network &network,
        const Static::Solution &x,
        const Static::SUMOAdapter &adapter
    );
};
// clang-format on

}  // namespace SUMO

namespace utils::stringify {
template<>
class stringify<SUMO::Routes::Flow::DepartPos> {
   public:
    static SUMO::Routes::Flow::DepartPos fromString(const std::string &s);

    static std::string toString(const SUMO::Routes::Flow::DepartPos &t);
};

template<>
class stringify<SUMO::Routes::Flow::DepartSpeed> {
   public:
    static SUMO::Routes::Flow::DepartSpeed fromString(const std::string &s);

    static std::string toString(const SUMO::Routes::Flow::DepartSpeed &t);
};
}  // namespace utils::stringify
