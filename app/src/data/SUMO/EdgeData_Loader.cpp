#include <stdexcept>

#include "Static/Solution.hpp"
#include "Static/supply/BPRNetwork.hpp"
#include "data/SUMO/EdgeData.hpp"

using namespace std;
using namespace SUMO;

typedef Static::Flow Flow;

EdgeData EdgeData::Loader<
    const NetworkTAZs&,
    const Static::BPRNetwork&,
    const Static::Solution&,
    const Static::SUMOAdapter&
>::load(
    const NetworkTAZs &sumo,
    const Static::BPRNetwork &network,
    const Static::Solution &x,
    const Static::SUMOAdapter &adapter
) {
    EdgeData ret;
    Interval &interval = ret.createInterval(0, 3600);

    const vector<SUMO::Network::Edge::ID> &sumoEdges = adapter.getSumoEdges();

    for(const SUMO::Network::Edge::ID &sumoEdgeID: sumoEdges) {
        const Static::BPRNetwork::Edge::ID eID = adapter.toEdge(sumoEdgeID);
        const Static::BPRNetwork::Edge    &e   = network.getEdge(eID);

        const Static::BPRNetwork::Node v = adapter.toNodes(sumoEdgeID).second;

        const double N = (double)sumo.network.getEdge(sumoEdgeID).lanes.size();

        const Flow cap = e.c;
        const Flow f   = x.getFlowInEdge(eID);
        const Time c   = e.calculateCongestion(x);

        double t0  = e.calculateCost(Static::SolutionBase());
        double fft = f * t0, t = f * e.calculateCost(x);
        for(const Static::Network::Edge *edge: network.getAdj(v)) {
            Flow f_ = x.getFlowInEdge(edge->id);
            fft += f_ * edge->calculateCost(Static::SolutionBase());
            t += f_ * edge->calculateCost(x);
        }

        Time d;
        if(f <= 0.0) {
            fft = t = t0;
            d       = 1.0;
        } else {
            fft /= f;
            t /= f;
            d = t / fft;
        }

        Interval::Edge &edge = interval.createEdge(sumoEdgeID);

        // clang-format off
        edge.attributes.setAttribute("capacity"         , cap);
        edge.attributes.setAttribute("capacityPerLane"  , cap / N);
        edge.attributes.setAttribute("flow"             , f);
        edge.attributes.setAttribute("flowPerLane"      , f / N);
        edge.attributes.setAttribute("congestion"       , c);
        edge.attributes.setAttribute("t0"               , t0);
        edge.attributes.setAttribute("fft"              , fft);
        edge.attributes.setAttribute("traveltime"       , t);
        edge.attributes.setAttribute("delay"            , d);
        edge.attributes.setAttribute("log_delay"        , log2(d));
        // clang-format on
    }

    return ret;
}
