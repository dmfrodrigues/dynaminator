#pragma once

#include <unordered_map>
#include <vector>

class Graph {

public:

    using Node = long;
    static const Node NODE_INVALID = -1;

    struct Edge {
        using Weight = double;
        static constexpr Weight WEIGHT_INF = 1.0e15;

        Node v;
        Weight w;
    };

private:
    std::vector<Node> nodes;
    std::unordered_map<Node, std::vector<Edge>> adj;

public:
    void addNode(Node u);
    void addEdge(Node u, Node v, Edge::Weight c);

    const std::vector<Node> &getNodes() const;
    const std::vector<Edge> &getAdj(Node u) const;
};
