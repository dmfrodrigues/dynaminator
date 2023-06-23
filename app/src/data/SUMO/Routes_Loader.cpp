#include <stdexcept>

#include "Dynamic/Env/Edge.hpp"
#include "Static/Solution.hpp"
#include "Static/supply/Network.hpp"
#include "data/SUMO/Network.hpp"
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

    map<pair<SUMO::TAZ::ID, SUMO::TAZ::ID>, vector<pair<Static::Flow, SUMO::Route>>> allRoutes;

    size_t       numberFlows = 0;
    Static::Flow maxFlow     = 0.0;

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
                route,
                0, 3600,
                shared_ptr<Flow::Policy>(
                    new Flow::PolicyVehsPerHour(f * 60.0 * 60.0)
                )
            );
            flow.setColor(colorRGB);
            flow.setFromTaz(fromTaz);
            flow.setToTaz(toTaz);
            flow.setDepartPos({.e = Flow::DepartPos::Enum::RANDOM_FREE});
            flow.setDepartSpeed({.e = Flow::DepartSpeed::Enum::RANDOM});
        }
    }

    return ret;
}

// clang-format off
Routes Routes::Loader<
    const list<reference_wrapper<const Dynamic::Env::Vehicle>> &,
    const SUMO::TAZs &,
    const Dynamic::SUMOAdapter &
>::load(
    const list<reference_wrapper<const Dynamic::Env::Vehicle>> &vehicles,
    const SUMO::TAZs &tazs,
    const Dynamic::SUMOAdapter &adapter
) {
    // clang-format on
    Routes ret;

    size_t i = 0;

    unordered_map<SUMO::Network::Edge::ID, SUMO::TAZ::ID> edge2tazSource;
    unordered_map<SUMO::Network::Edge::ID, SUMO::TAZ::ID> edge2tazSink;
    for(const auto &[_, taz]: tazs) {
        for(const SUMO::TAZ::Source &source: taz.sources) {
            if(edge2tazSource.count(source.id)) {
                // clang-format off
                // throw logic_error(
                cerr << "[WARN] " <<
                    "Routes::Loader<>::load: Source edge " + source.id +
                    " already assigned to TAZ " + edge2tazSource[source.id] +
                    ", cannot be assigned to TAZ " + taz.id + " as well"
                << endl;
                // );
                // clang-format on
            }
            edge2tazSource[source.id] = taz.id;
        }
        for(const SUMO::TAZ::Sink &sink: taz.sinks) {
            if(edge2tazSink.count(sink.id)) {
                // clang-format off
                // throw logic_error(
                cerr << "[WARN] " <<
                    "Routes::Loader<>::load: Sink edge " + sink.id +
                    " already assigned to TAZ " + edge2tazSink[sink.id] +
                    ", cannot be assigned to TAZ " + taz.id + " as well"
                << endl;
                // );
                // clang-format on
            }
            edge2tazSink[sink.id] = taz.id;
        }
    }

    for(const Dynamic::Env::Vehicle &vehicle: vehicles) {
        Route route;
        for(const Dynamic::Env::Lane &lane: vehicle.path) {
            SUMO::Network::Edge::ID edgeID = adapter.toSumoEdge(lane.edge.id);

            if(!route.empty() && route.back() == edgeID)
                continue;

            route.push_back(edgeID);
        }

        Vehicle &veh = ret.createVehicle(
            to_string(vehicle.id),
            route,
            vehicle.depart
        );

        float h = 360.0f * float(i) / float(vehicles.size());
        float v = 100.0f;
        float s = 100.0f;

        color::hsv<float> colorHSV({h, s, v});
        color::rgb<float> colorRGB;
        colorRGB = colorHSV;

        veh.setColor(colorRGB);
        veh.setFromTaz(edge2tazSource.at(adapter.toSumoEdge(vehicle.from.id)));
        veh.setToTaz(edge2tazSink.at(adapter.toSumoEdge(vehicle.to.id)));

        ++i;
    }

    return ret;
}
