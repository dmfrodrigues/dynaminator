#include "Static/Demand.hpp"

#include <iostream>

using namespace std;
using namespace Static;

typedef Network::Node Node;
typedef Network::Flow Flow;

void Demand::addDemand(Node u, Node v, Flow f) {
    flows[u][v] += f;
}

vector<Node> Demand::getStartNodes() const {
    vector<Node> ret;
    ret.reserve(flows.size());

    for (const auto &[u, _]: flows)
        ret.push_back(u);

    return ret;
}

vector<Node> Demand::getDestinations(Node u) const {
    const auto &dest = flows.at(u);

    vector<Node> ret;
    ret.reserve(dest.size());

    for (const auto &[v, _]: dest)
        ret.push_back(v);

    return ret;
}

Flow Demand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}

Flow Demand::getTotalDemand() const {
    Flow f = 0;
    for(const auto &p1 : flows){
        for(const auto &p2 : p1.second){
            f += p2.second;
        }
    }
    return f;
}

// clang-format off
Demand Demand::Loader<
    const VISUM::OFormatDemand &,
    const SUMOAdapter &    
>::load(
    const VISUM::OFormatDemand &oDemand,
    const SUMOAdapter           &adapter
) {
    // clang-format on

    Demand demand;

    const auto &startNodes = oDemand.getStartNodes();
    for (const auto &u : startNodes) {
        const auto &destinations = oDemand.getDestinations(u);
        for (const auto &v : destinations) {
            try {
                demand.addDemand(
                    adapter.toTAZNode(u).first,
                    adapter.toTAZNode(v).second,
                    oDemand.getDemand(u, v)/(oDemand.getTo() - oDemand.getFrom())
                );
            } catch(const out_of_range &e){
                cerr << e.what() << " | "
                    << "u=" << u << " (" << u << "), "
                    << "v=" << v << " (" << v << "), "
                    << "f=" << oDemand.getDemand(u, v) << endl;
            }
        }
    }

    return demand;
}
