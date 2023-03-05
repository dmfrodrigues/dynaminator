#pragma once

#include <unordered_map>
#include <vector>

class Graph {

public:

    struct Edge {
        long to;
        double cost;
    };

private:

    std::unordered_map<long, std::vector<Edge>> adj;

    void addNode(long u);
    void addEdge(long u, long v, double c);

    const std::vector<Edge> &getAdj(long u) const;
};
