#include "Static/SUMOAdapter.hpp"

#include <iostream>

using namespace std;
using namespace Static;

typedef Network::Node               Node;
typedef Network::Edge::ID           Edge;
typedef SUMO::Network::Junction::ID SumoJunction;
typedef SUMO::TAZ::ID               TAZ;
typedef SUMO::Network::Edge::ID     SumoEdge;

// Node SUMOAdapter::addNode() {
//     Edge b = nextNode++;
//     return b;
// }

pair<Edge, pair<Node, Node>> SUMOAdapter::addSumoEdge(const SumoEdge &a) {
    Edge                         e = nextEdge++;
    Node                         u = nextNode++, v = nextNode++;
    pair<Edge, pair<Node, Node>> ret(e, {u, v});

    edge2sumoEdge[ret.first]         = a;
    sumoEdge2edge[a]                 = ret.first;
    sumoEdge2nodes[a]                = ret.second;
    node2sumoEdge[ret.second.first]  = a;
    node2sumoEdge[ret.second.second] = a;
    return ret;
}
pair<Node, Node> SUMOAdapter::addSumoTAZ(const TAZ &a) {
    Node b          = nextNode++;
    Node c          = nextNode++;
    node2sumoTAZ[b] = a;
    node2sumoTAZ[c] = a;
    sumoTAZ2node[a] = pair<Node, Node>(b, c);

    return pair<Node, Node>(b, c);
}

Edge SUMOAdapter::addEdge() {
    return nextEdge++;
}

bool SUMOAdapter::isEdge(const SumoEdge &a) const {
    return sumoEdge2edge.count(a);
}

const pair<Node, Node> &SUMOAdapter::toTAZNode(const TAZ &a) const {
    return sumoTAZ2node.at(a);
}
const TAZ &SUMOAdapter::toSumoTAZ(const Node &a) const {
    return node2sumoTAZ.at(a);
}

const Edge &SUMOAdapter::toEdge(const SumoEdge &a) const {
    return sumoEdge2edge.at(a);
}
const SumoEdge &SUMOAdapter::toSumoEdge(const Edge &a) const {
    return edge2sumoEdge.at(a);
}
const pair<Node, Node> &SUMOAdapter::toNodes(const SumoEdge &a) const {
    return sumoEdge2nodes.at(a);
}

bool SUMOAdapter::isSumoEdge(const Edge &a) const {
    return edge2sumoEdge.count(a) > 0;
}

vector<SumoEdge> SUMOAdapter::getSumoEdges() const {
    vector<SumoEdge> ret;
    ret.reserve(sumoEdge2edge.size());
    for(const auto &p: sumoEdge2edge) {
        ret.push_back(p.first);
    }
    return ret;
}

SumoEdge SUMOAdapter::fromNodeToSumoEdge(const Node &a) const {
    return node2sumoEdge.at(a);
}

void SUMOAdapter::clear() {
    edge2sumoEdge.clear();
    sumoEdge2edge.clear();
    sumoEdge2nodes.clear();
    node2sumoTAZ.clear();
    sumoTAZ2node.clear();
    nextNode = 0;
    nextEdge = 0;
}

void SUMOAdapter::dump() const {
    cerr << "SUMOAdapter::dump()" << endl;
    cerr << "edge2sumoEdge:" << endl;
    for(const auto &[edgeID, sumoEdgeID]: edge2sumoEdge) {
        const auto &[u, v] = sumoEdge2nodes.at(sumoEdgeID);
        cerr << "Edge " << edgeID
             << " (nodes " << u << ", " << v
             << ") → SUMO edge " << sumoEdgeID << endl;
    }
}
