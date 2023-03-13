#include "static/StaticDemand.hpp"

#include <iostream>

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Flow Flow;

void StaticDemand::addDemand(Node u, Node v, Flow f) {
    flows[u][v] += f;
}

vector<Node> StaticDemand::getStartNodes() const {
    vector<Node> ret;
    ret.reserve(flows.size());

    for (const auto &p : flows)
        ret.push_back(p.first);

    return ret;
}

vector<Node> StaticDemand::getDestinations(Node u) const {
    const auto &dest = flows.at(u);

    vector<Node> ret;
    ret.reserve(dest.size());

    for (const auto &p : dest)
        ret.push_back(p.first);

    return ret;
}

Flow StaticDemand::getDemand(Node u, Node v) const {
    return flows.at(u).at(v);
}

Flow StaticDemand::getTotalDemand() const {
    Flow f = 0;
    for(const auto &p1 : flows){
        for(const auto &p2 : p1.second){
            f += p2.second;
        }
    }
    return f;
}

StaticDemand StaticDemand::fromOFormat(
    const OFormatDemand &oDemand,
    const std::unordered_map<
        SumoNetwork::Junction::Id,
        std::pair<StaticNetwork::Node, StaticNetwork::Node> > &str2id_taz) {
    StaticDemand demand;

    const auto &startNodes = oDemand.getStartNodes();
    for (const auto &u : startNodes) {
        const auto &destinations = oDemand.getDestinations(u);
        for (const auto &v : destinations) {
            try {
                demand.addDemand(
                    str2id_taz.at(u).first,
                    str2id_taz.at(v).second,
                    oDemand.getDemand(u, v)/(oDemand.getTo() - oDemand.getFrom())
                );
            } catch(const out_of_range &e){
                // cerr << e.what() << " | "
                //     << "u=" << u << " (" << str2id_taz.count(u) << "), "
                //     << "v=" << v << " (" << str2id_taz.count(v) << "), "
                //     << "f=" << oDemand.getDemand(u, v) << endl;
            }
        }
    }

    return demand;
}
