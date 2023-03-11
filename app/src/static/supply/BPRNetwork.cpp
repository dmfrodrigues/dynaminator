#include "static/supply/BPRNetwork.hpp"

#include <cmath>
#include <iostream>

using namespace std;

typedef BPRNetwork::Node Node;
typedef BPRNetwork::Edge Edge;
typedef BPRNetwork::Flow Flow;
typedef BPRNetwork::Cost Cost;

BPRNetwork::BPRNetwork(double alpha_, double beta_):alpha(alpha_), beta(beta_){}

void BPRNetwork::addNode(Node u){
    adj[u];
}

void BPRNetwork::addEdge(Edge::Id id, Node u, Node v, Time t0, Capacity c){
    CustomEdge *e = new CustomEdge{id, v, t0, c};
    adj[u].push_back(e);
    edges[id] = e;
}

std::vector<Node> BPRNetwork::getNodes() const {
    vector<Node> ret;
    ret.reserve(adj.size());
    for(const auto &p: adj)
        ret.push_back(p.first);
    return ret;
}

std::vector<Edge *> BPRNetwork::getAdj(Node u) const {
    const auto &v = adj.at(u);
    return vector<Edge *>(v.begin(), v.end());
}

Cost BPRNetwork::calculateCost(Edge::Id id, Flow f) const {
    CustomEdge *e = edges.at(id);
    return e->t0 * (1.0 + alpha * pow(f/e->c, beta));
}

BPRNetwork* BPRNetwork::fromSumoNetwork(const SumoNetwork &sumoNetwork){
    BPRNetwork *network = new BPRNetwork();
    
    const vector<SumoNetwork::Junction> &junctions = sumoNetwork.getJunctions();
    for(const SumoNetwork::Junction &j: junctions){
        network->addNode(j.id);
    }
    
    const std::vector<SumoNetwork::Edge> &edges = sumoNetwork.getEdges();
    for(const SumoNetwork::Edge &e: edges){
        if(e.function == SumoNetwork::Edge::Function::INTERNAL) continue;
        if(
            e.from == SumoNetwork::Junction::INVALID ||
            e.to   == SumoNetwork::Junction::INVALID
        ){
            cerr << "Edge " << e.id << " is invalid, failed to build BPRNetwork" << endl;
            delete network;
            return nullptr;
        }

        double length = 0, averageSpeed = 0;
        for(const auto &p: e.lanes){
            length += p.second.length;
            averageSpeed += p.second.speed;
        }
        length /= double(e.lanes.size());
        averageSpeed /= double(e.lanes.size());

        double freeFlowSpeed = averageSpeed;
        double freeFlowTime = length/freeFlowSpeed;

        // Estimated using Greenshields' model. Sheffi (1985), p. 350.
        // This roughly agrees with the Highway Capacity Manual (p. 10-24, 15-9)
        const double jamDensity = 1.0/8.0;
        double capacity = 0.25 * freeFlowSpeed * jamDensity;

        network->addEdge(e.id, e.from, e.to, freeFlowTime, capacity);
    }

    return network;
}
