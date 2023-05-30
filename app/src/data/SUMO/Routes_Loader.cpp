#include "Static/Solution.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/Routes.hpp"
#include "utils/stringify.hpp"

using namespace std;
using namespace SUMO;

using utils::stringify::stringify;

// clang-format off
Routes Routes::Loader<
    const Static::Network &,
    const Static::Solution &,
    const Static::SUMOAdapter &
>::load(
    const Static::Network &network,
    const Static::Solution &x,
    const Static::SUMOAdapter &adapter
) {
    // clang-format on
    Routes ret;

    const Static::Solution::Routes &routes = x.getRoutes();

    map<pair<SUMO::TAZ::ID, SUMO::TAZ::ID>, vector<pair<Static::Network::Flow, SUMO::Route>>> allRoutes;

    size_t                numberFlows = 0;
    Static::Network::Flow maxFlow     = 0.0;

    for(const auto &[path, flow]: routes) {
        SUMO::Route route;
        for(const Static::Network::Edge::ID &eid: path) {
            if(adapter.isSumoEdge(eid))
                route.push_back(adapter.toSumoEdge(eid));
        }
        const SUMO::TAZ::ID &fromTaz = adapter.toSumoTAZ(network.getEdge(*path.begin()).u);
        const SUMO::TAZ::ID &toTaz   = adapter.toSumoTAZ(network.getEdge(*path.rbegin()).v);

        allRoutes[{fromTaz, toTaz}].push_back({flow, route});
        ++numberFlows;
        maxFlow = max(maxFlow, flow);
    }

    size_t flowID = 0;
    for(const auto &[fromTo, fromToRoutes]: allRoutes) {
        const auto &[fromTaz, toTaz] = fromTo;

        const float MIN_INTENSITY     = 0.5;
        const float MAX_INTENSITY     = 1.5;
        const float DEFAULT_INTENSITY = 1.0;

        for(size_t i = 0; i < fromToRoutes.size(); ++i) {
            const auto &[f, route] = fromToRoutes[i];

            float intensity = (fromToRoutes.size() <= 1 ? DEFAULT_INTENSITY : MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * float(i) / float(fromToRoutes.size() - 1));

            float h = 360.0f * float(flowID) / float(numberFlows);
            float v = 100.0f * min(intensity, 1.0f);
            float s = 100.0f * (1.0f - max(intensity - 1.0f, 0.0f));

            color::hsv<float> colorHSV({h, s, v});
            color::rgb<float> colorRGB;
            colorRGB = colorHSV;

            Flow &flow = ret.createFlow(
                stringify<size_t>::toString(flowID++),
                0, 3600,
                shared_ptr<Flow::Policy>(
                    new Flow::PolicyVehsPerHour(f * 60.0 * 60.0)
                ),
                route
            );
            flow.setColor(colorRGB);
            flow.setFromTaz(fromTaz);
            flow.setToTaz(toTaz);
            flow.setDepartPos  ({.e = Flow::DepartPos::Enum::RANDOM_FREE});
            flow.setDepartSpeed({.e = Flow::DepartSpeed::Enum::RANDOM});
        }
    }

    return ret;
}
