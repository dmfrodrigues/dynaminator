#include "data/SumoAdapterStatic.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Edge::ID Edge;
typedef SUMO::Network::Junction::ID SumoJunction;
typedef SumoTAZs::TAZ::ID TAZ;
typedef SUMO::Network::Edge::ID SumoEdge;

Node SumoAdapterStatic::addNode() {
    Edge b = nextNode++;
    return b;
}

Edge SumoAdapterStatic::addSumoEdge(const SumoEdge &a) {
    Edge b = nextEdge++;
    edge2sumoEdge[b] = a;
    sumoEdge2edge[a] = b;
    return b;
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
