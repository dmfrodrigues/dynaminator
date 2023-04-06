#include "shortest-path/DijkstraCuda.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <queue>
#include <utility>

#include "structs/BinaryHeapCuda.hpp"

using namespace std;

typedef Graph::Node Node;
typedef Graph::Edge::Weight Weight;
typedef Graph::Edge Edge;
template <class K, class V>
using umap = std::unordered_map<K, V>;
typedef umap<Node, Weight> dist_t;
typedef umap<Node, Node> prev_t;
typedef BinaryHeapCuda<cuda::pair<Weight, Node>> MinPriorityQueue;
typedef cuda::vector<MinPriorityQueue::Element *> Elements;
typedef std::chrono::high_resolution_clock hrc;

void DijkstraCuda::initialize(const Graph *G, const list<Node> &s) {
    const vector<Node> &nodes = G->getNodes();

    adj.clear();
    size_t numberNodes = (nodes.empty() ? 1 : *max_element(nodes.begin(), nodes.end()) + 1);
    adj.resize(numberNodes, {-1, -1});

    edges.clear();
    size_t numberEdges = 0;
    for (const Node &u : nodes) numberEdges += G->getAdj(u).size();
    edges.reserve(numberEdges);

    size_t edgeIdx = 0;
    for (const Node &u : nodes) {
        const auto &es = G->getAdj(u);
        adj[u] = pair<uint32_t, uint32_t>(edgeIdx, edgeIdx + es.size());
        edges.insert(edges.end(), es.begin(), es.end());
        edgeIdx += es.size();
    }
    assert(edgeIdx == numberEdges);

    numberStartNodes = s.size();
    cudaErrchk(cudaMallocManaged(&startNodes, numberStartNodes * sizeof(Node)));
    copy(s.begin(), s.end(), startNodes);

    cudaErrchk(cudaMallocManaged(&prev, numberNodes * sizeof(Edge *)));
    cudaErrchk(cudaMallocManaged(&dist, numberNodes * sizeof(Weight *)));
    fill(prev, prev + numberNodes, nullptr);
    fill(dist, dist + numberNodes, nullptr);
    for (const Node &u : s) {
        cudaErrchk(cudaMallocManaged(&prev[u], numberNodes * sizeof(Edge)));
        cudaErrchk(cudaMallocManaged(&dist[u], numberNodes * sizeof(Weight)));
        fill(prev[u], prev[u] + numberNodes, Graph::EDGE_INVALID);
        fill(dist[u], dist[u] + numberNodes, Edge::WEIGHT_INF);
    }
}

union EdgeInt4 {
    int4 i;
    Edge e;
};

union AdjInt2 {
    int2 i;
    cuda::pair<uint32_t, uint32_t> p;
};

__device__ void runDijkstra(
    size_t numberNodes, size_t numberEdges,
    cudaTextureObject_t edges,
    cudaTextureObject_t adj,
    Node s,
    Elements &elements,
    MinPriorityQueue &Q,
    Edge *prev,
    Weight *dist) {
    dist[s] = 0;
    auto el = Q.push({0, s});
    elements[s] = &el;
    while (!Q.empty()) {
        cuda::pair<Weight, Node> p = Q.top(); Q.pop();
        Node u = p.second;
        Weight du = p.first;
        AdjInt2 ai2 = {.i = tex1Dfetch<int2>(adj, u)};
        for (uint32_t i = ai2.p.first; i < ai2.p.second; ++i) {
            EdgeInt4 ei4 = {.i = tex1Dfetch<int4>(edges, i)};
            const Edge &e = ei4.e;
            Weight c_ = du + e.w;
            Weight &distV = dist[e.v];
            if (c_ < distV) {
                if (elements[e.v])
                    elements[e.v]->decreaseKey({c_, e.v});
                else
                    elements[e.v] = &Q.push({c_, e.v});
                distV = c_;
                prev[e.v] = e;
            }
        }
    }
}

__global__ void runDijkstraKernel(
    size_t numberStartNodes, Node *startNodes,
    size_t numberNodes, size_t numberEdges,
    cudaTextureObject_t edges,
    cudaTextureObject_t adj,
    cuda::vector<Elements *> &elements,
    cuda::vector<MinPriorityQueue *> &Q,
    Edge **prev,
    Weight **dist) {
    int a = blockIdx.x * blockDim.x + threadIdx.x;
    int b = blockIdx.y * blockDim.y + threadIdx.y;
    const int BMAX = blockDim.y * gridDim.y;
    int i = a * BMAX + b;
    if (i >= numberStartNodes)
        return;
    Node s = startNodes[i];
    runDijkstra(numberNodes, numberEdges, edges, adj, s, *elements.at(i), *Q.at(i), prev[s], dist[s]);
}

void DijkstraCuda::run() {
    const size_t &numberEdges = edges.size();
    const size_t &numberNodes = adj.size();

    // Inspired by https://stackoverflow.com/q/55348493/12283316
    // Edges texture
    int4 *edgesArr;
    cudaTextureObject_t edgesTex;
    assert(sizeof(Edge) == sizeof(int4));
    const size_t edgeSize = sizeof(int4) * numberEdges;
    cudaErrchk(cudaMalloc(&edgesArr, edgeSize));
    cudaMemcpy(edgesArr, &edges[0], edgeSize, cudaMemcpyHostToDevice);
    struct cudaResourceDesc edgesResDesc;
    edgesResDesc.resType = cudaResourceTypeLinear;
    edgesResDesc.res.linear.devPtr = edgesArr;
    edgesResDesc.res.linear.sizeInBytes = edgeSize;
    edgesResDesc.res.linear.desc = cudaCreateChannelDesc<int4>();
    struct cudaTextureDesc edgesTexDesc = {};
    cudaErrchk(cudaCreateTextureObject(&edgesTex, &edgesResDesc, &edgesTexDesc, NULL));

    // Adj texture
    int2 *adjArr;
    cudaTextureObject_t adjTex;
    assert(sizeof(cuda::pair<uint32_t, uint32_t>) == sizeof(int2));
    const size_t adjSize = sizeof(int2) * numberNodes;
    cudaErrchk(cudaMalloc(&adjArr, adjSize));
    cudaMemcpy(adjArr, &adj[0], adjSize, cudaMemcpyHostToDevice);
    struct cudaResourceDesc adjResDesc;
    adjResDesc.resType = cudaResourceTypeLinear;
    adjResDesc.res.linear.devPtr = adjArr;
    adjResDesc.res.linear.sizeInBytes = adjSize;
    adjResDesc.res.linear.desc = cudaCreateChannelDesc<int2>();
    struct cudaTextureDesc adjTexDesc = {};
    cudaErrchk(cudaCreateTextureObject(&adjTex, &adjResDesc, &adjTexDesc, NULL));

    // Elements
    cuda::vector<Elements *> *elements = cuda::vector<Elements *>::constructShared(numberStartNodes);
    for (size_t i = 0; i < numberStartNodes; ++i)
        elements->emplace_back(Elements::constructShared(numberNodes, numberNodes, nullptr));

    // Q
    cuda::vector<MinPriorityQueue *> *Q = cuda::vector<MinPriorityQueue *>::constructShared(numberStartNodes);
    for (size_t i = 0; i < numberStartNodes; ++i)
        Q->emplace_back(MinPriorityQueue::constructShared(numberNodes));

    const size_t &N = numberStartNodes;
    dim3 threadsPerBlock(16, 8);
    dim3 numBlocks(
        (N + threadsPerBlock.x - 1) / threadsPerBlock.x,
        (N + threadsPerBlock.y - 1) / threadsPerBlock.y);
    runDijkstraKernel<<<numBlocks, threadsPerBlock>>>(
        numberStartNodes, startNodes,
        numberNodes, numberEdges,
        edgesTex, adjTex,
        *elements, *Q,
        prev, dist);
    cudaErrchk(cudaPeekAtLastError());
    cudaErrchk(cudaDeviceSynchronize());

    elements->destroyShared();
    Q->destroyShared();

    cudaErrchk(cudaFree(edgesArr));
    cudaErrchk(cudaFree(adjArr));
}

Edge DijkstraCuda::getPrev(Node s, Node d) const {
    const size_t &numberNodes = adj.size();
    if (s >= numberNodes || prev[s] == nullptr)
        throw out_of_range("s is not a valid start node");
    if (d >= numberNodes)
        throw out_of_range("d is not a valid destination node");
    return prev[s][d];
}

Weight DijkstraCuda::getPathWeight(Node s, Node d) const {
    const size_t &numberNodes = adj.size();
    if (s >= numberNodes || dist[s] == nullptr)
        throw out_of_range("s is not a valid start node");
    if (d >= numberNodes)
        throw out_of_range("d is not a valid destination node");
    return dist[s][d];
}

bool DijkstraCuda::hasVisited(Node s, Node u) const {
    return getPathWeight(s, u) != Edge::WEIGHT_INF;
}

DijkstraCuda::~DijkstraCuda() {
}
