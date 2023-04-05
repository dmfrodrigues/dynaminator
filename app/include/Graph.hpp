#pragma once

#include <unordered_map>
#include <vector>

class Graph {

public:

    typedef long Node;
    static const Node NODE_INVALID = -1;

    struct Edge {
        typedef double Weight;
        typedef long ID;

        static const Weight WEIGHT_INF;

        ID id;
        Node u;
        Node v;
        Weight w;
    };

    static const Edge EDGE_INVALID;

    typedef std::vector<Graph::Edge> Path;

private:
    std::unordered_map<Node, std::vector<Edge>> adj;

public:
    void addNode(Node u);
    void addEdge(Edge::ID id, Node u, Node v, Edge::Weight c);

    std::vector<Node> getNodes() const;
    const std::vector<Edge> &getAdj(Node u) const;
};
