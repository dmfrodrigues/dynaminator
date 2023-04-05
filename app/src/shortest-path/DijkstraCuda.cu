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
typedef cuda::vector<MinPriorityQueue::Element*> Elements;
typedef std::chrono::high_resolution_clock hrc;

void DijkstraCuda::initialize(const Graph *G, const list<Node> &s) {
    const vector<Node> &nodes = G->getNodes();

    numberNodes = (nodes.empty() ? 1 : *max_element(nodes.begin(), nodes.end()) + 1);
    cudaErrchk(cudaMallocManaged(&adj, numberNodes * sizeof(pair<Edge::ID, Edge::ID>)));

    numberEdges = 0;
    for (const Node &u : nodes) numberEdges += G->getAdj(u).size();
    cudaErrchk(cudaMallocManaged(&edges, numberEdges * sizeof(Edge)));

    size_t edgeIdx = 0;
    for (const Node &u : nodes) {
        const auto &es = G->getAdj(u);
        adj[u] = pair<Edge::ID, Edge::ID>(edgeIdx, edgeIdx + es.size());
        copy(es.begin(), es.end(), &edges[edgeIdx]);
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

__device__ __host__
void runDijkstra(
    size_t numberNodes, size_t numberEdges,
    const Edge *edges, const pair<Edge::ID, Edge::ID> *adj,
    Node s,
    Elements &elements,
    MinPriorityQueue &Q,
    Edge *prev,
    Weight *dist) {

    dist[s] = 0;
    auto el = Q.push({0, s});
    elements[s] = &el;
    while (!Q.empty()) {
        Node u = Q.top().second;
        Q.pop();
        for (size_t i = adj[u].first; i < adj[u].second; ++i) {
            const Edge &e = edges[i];
            Weight c_ = dist[u] + e.w;
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

__global__
void runDijkstraKernel(
    size_t numberStartNodes, Node *startNodes,
    size_t numberNodes, size_t numberEdges,
    const Edge *edges, const pair<Edge::ID, Edge::ID> *adj,
    cuda::vector<Elements*> &elements,
    cuda::vector<MinPriorityQueue*> &Q,
    Edge **prev,
    Weight **dist
) {
    int a = blockIdx.x * blockDim.x + threadIdx.x;
    int b = blockIdx.y * blockDim.y + threadIdx.y;
    const int BMAX = blockDim.y * gridDim.y;
    int i = a * BMAX + b;
    if(i >= numberStartNodes)
        return;
    Node s = startNodes[i];
    // for(int i = 0; i < numberStartNodes; ++i)
    //     printf("start node %d\n", startNodes[i]);
    // printf("i=%d, s=%d, startNodes[i]=%d\n", i, s, startNodes[i]);
    runDijkstra(numberNodes, numberEdges, edges, adj, s, *elements.at(i), *Q.at(i), prev[s], dist[s]);
}

void DijkstraCuda::run() {
    cuda::vector<Elements*> *elements = cuda::vector<Elements*>::constructShared(numberStartNodes);
    for(size_t i = 0; i < numberStartNodes; ++i)
        elements->emplace_back(Elements::constructShared(numberNodes, numberNodes, nullptr));

    cuda::vector<MinPriorityQueue*> *Q = cuda::vector<MinPriorityQueue*>::constructShared(numberStartNodes);
    for(size_t i = 0; i < numberStartNodes; ++i)
        Q->emplace_back(MinPriorityQueue::constructShared(numberNodes));

    // Elements *elements = Elements::constructShared(numberNodes, numberNodes, nullptr);
    // MinPriorityQueue *Q = MinPriorityQueue::constructShared(numberNodes);

    // for (size_t i = 0; i < numberStartNodes; ++i) {
    //     const Node &s = startNodes[i];
    //     runDijkstra(numberNodes, numberEdges, edges, adj, s, elements->at(i), Q->at(i), prev[s], dist[s]);
    // }

    const size_t &N = numberStartNodes;
    dim3 threadsPerBlock(16, 8);
    dim3 numBlocks(
        (N + threadsPerBlock.x - 1)/threadsPerBlock.x,
        (N + threadsPerBlock.y - 1)/threadsPerBlock.y
    );
    runDijkstraKernel<<<numBlocks, threadsPerBlock>>>(
        numberStartNodes, startNodes,
        numberNodes, numberEdges,
        edges, adj,
        *elements, *Q,
        prev, dist
    );
    cudaErrchk(cudaPeekAtLastError());
    cudaErrchk(cudaDeviceSynchronize());
    cudaDeviceSynchronize();

    elements->destroyShared();
    Q->destroyShared();
}

Edge DijkstraCuda::getPrev(Node s, Node d) const {
    if (s >= numberNodes || prev[s] == nullptr)
        throw out_of_range("s is not a valid start node");
    if (d >= numberNodes)
        throw out_of_range("d is not a valid destination node");
    return prev[s][d];
}

Weight DijkstraCuda::getPathWeight(Node s, Node d) const {
    if (s >= numberNodes || dist[s] == nullptr)
        throw out_of_range("s is not a valid start node");
    if (d >= numberNodes)
        throw out_of_range("d is not a valid destination node");
    return dist[s][d];
}

bool DijkstraCuda::hasVisited(Node s, Node u) const {
    return getPathWeight(s, u) != Edge::WEIGHT_INF;
}

DijkstraCuda::~DijkstraCuda(){
    cudaErrchk(cudaFree(adj));
    cudaErrchk(cudaFree(edges));
}
