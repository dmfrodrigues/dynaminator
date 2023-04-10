#include "data/SumoAdapterStatic.hpp"

#include <iostream>

using namespace std;

typedef Static::Network::Node Node;
typedef Static::Network::Edge::ID Edge;
typedef SUMO::Network::Junction::ID SumoJunction;
typedef SUMO::TAZ::ID TAZ;
typedef SUMO::Network::Edge::ID SumoEdge;

// Node SumoAdapterStatic::addNode() {
//     Edge b = nextNode++;
//     return b;
// }

pair<Edge, pair<Node, Node>> SumoAdapterStatic::addSumoEdge(const SumoEdge &a) {
    Edge e = nextEdge++;
    Node u = nextNode++, v = nextNode++;
    pair<Edge, pair<Node, Node>> ret(e, {u, v});

    edge2sumoEdge[ret.first] = a;
    sumoEdge2edge[a] = ret.first;
    sumoEdge2nodes[a] = ret.second;
    return ret;
}
pair<Node, Node> SumoAdapterStatic::addSumoTAZ(const TAZ &a){
    Node b = nextNode++;
    Node c = nextNode++;
    node2sumoTAZ[b] = a;
    node2sumoTAZ[c] = a;
    sumoTAZ2node[a] = pair<Node, Node>(b, c);
    
    return pair<Node, Node>(b, c);
}

Edge SumoAdapterStatic::addEdge() {
    return nextEdge++;
}

bool SumoAdapterStatic::isEdge(const SumoEdge &a) const {
    return sumoEdge2edge.count(a);
}

const pair<Node, Node> &SumoAdapterStatic::toTAZNode(const TAZ &a) const {
    return sumoTAZ2node.at(a);
}
const TAZ &SumoAdapterStatic::toSumoTAZ(const Node &a) const {
    return node2sumoTAZ.at(a);
}

const Edge &SumoAdapterStatic::toEdge(const SumoEdge &a) const {
    return sumoEdge2edge.at(a);
}
const SumoEdge &SumoAdapterStatic::toSumoEdge(const Edge &a) const {
    return edge2sumoEdge.at(a);
}
const pair<Node, Node> &SumoAdapterStatic::toNodes(const SumoEdge &a) const {
    return sumoEdge2nodes.at(a);
}

vector<SumoEdge> SumoAdapterStatic::getSumoEdges() const {
    vector<SumoEdge> ret;
    ret.reserve(sumoEdge2edge.size());
    for(const auto &p: sumoEdge2edge){
        ret.push_back(p.first);
    }
    return ret;
}
