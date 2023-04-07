#include "static/algos/FrankWolfe.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <iomanip>

#include "convex/QuadraticSolver.hpp"
#include "shortest-path/DijkstraMany.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Edge Edge;
typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Cost Cost;

FrankWolfe::FrankWolfe(StaticProblem prob)
    : problem(prob) {}

void FrankWolfe::setStartingSolution(const StaticSolution &startingSolution) {
    xn = startingSolution;
    zn = problem.supply.evaluate(xn);
}

void FrankWolfe::setStopCriteria(Cost e) {
    epsilon = e;
}

void FrankWolfe::setIterations(int it){
    iterations = it;
}

StaticSolution FrankWolfe::solve() {
    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.
    cout
        << "FW algorithm\n"
        << "it\talpha\tzn\tdelta\tlowerBound\tAbsGap\tRelGap\n";
    cout << fixed << setprecision(9);
    Flow znPrev = zn;
    for (int it = 0; it < iterations; ++it) {
        Cost delta = znPrev - zn;
        Cost absoluteGap = zn - lowerBound;
        Cost relativeGap = absoluteGap/zn;
        cout << it
            << "\t" << alpha
            << "\t" << zn
            << "\t" << delta
            << "\t" << lowerBound
            << "\t" << absoluteGap
            << "\t" << relativeGap
            << endl;

        znPrev = zn;

        StaticSolutionBase xstar = step1();

        xn = step2(xstar);
        zn = problem.supply.evaluate(xn);

        if (absoluteGap <= epsilon) {
            cout << "FW: Met relative gap criteria. Stopping" << endl;
            return xn;
        }
    }

    return xn;
}

StaticSolutionBase FrankWolfe::step1() {
    // TODO: duplicate code with AllOrNothing::solve() (AllOrNothing.cpp)
    StaticSolutionBase xstar;

    Graph G = problem.supply.toGraph(xn);

    const vector<Node> startNodes = problem.demand.getStartNodes();

    DijkstraMany shortestPaths;
    shortestPaths.initialize(&G, startNodes);
    shortestPaths.run();

    for (const Node &u : startNodes) {
        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for (const Node &v : endNodes) {
            Graph::Path path = shortestPaths.getPath(u, v);

            if (path.size() == 1 && path.front().id == Graph::EDGE_INVALID.id)
                throw logic_error("Could not find path " + to_string(u) + "->" + to_string(v));

            StaticNetwork::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for (const Graph::Edge &e : path)
                pathNetwork.push_back(e.id);

            Flow f = problem.demand.getDemand(u, v);
            xstar.addPath(pathNetwork, f);
        }
    }

    // Update lower bound
    Cost zApprox = zn;
    unordered_set<Edge::ID> edges;
    const unordered_set<Edge::ID> &xnEdges = xn.getEdges();
    const unordered_set<Edge::ID> &xstarEdges = xstar.getEdges();
    edges.insert(xnEdges.begin(), xnEdges.end());
    edges.insert(xstarEdges.begin(), xstarEdges.end());
    for(const Edge::ID &eid: edges){
        Flow xna = xn.getFlowInEdge(eid);
        Flow xstara = xstar.getFlowInEdge(eid);
        zApprox += problem.supply.calculateCost(eid, xna) * (xstara - xna);
    }
    lowerBound = max(lowerBound, zApprox);

    return xstar;
}

StaticSolution FrankWolfe::step2(const StaticSolution &xstar) {
    // TODO: allow to tune this value of epsilon
    const ConvexSolver::Var EPSILON = 1e-6;
    unique_ptr<ConvexSolver> solver;
    {
        QuadraticSolver *is = new QuadraticSolver();
        // is->setSolutions(0, 1, 0.5);
        /*
         * Make initial solutions adaptive; e.g., x1 = 0, x2 is the geometric
         * mean of the final values of alpha so far, and x3 = 2 * x2.
         */
        is->setSolutions(0, 0.1, 0.2);
        is->setStopCriteria(EPSILON);

        solver = unique_ptr<ConvexSolver>(is);
    }
    ConvexSolver::Problem p = [
        &problem = as_const(problem),
        &xn = as_const(xn),
        &xstar = as_const(xstar)
    ](ConvexSolver::Var a) {
        StaticSolution x = StaticSolution::interpolate(xn, xstar, a);
        StaticNetwork::Cost c = problem.supply.evaluate(x);
        return c;
    };
    solver.get()->setProblem(p);

    alpha = solver.get()->solve();
    if(alpha < 0.0){
        cerr << "alpha (" << alpha << ") < 0, assuming alpha = 0" << endl;
        alpha = 0.0;
    } else if(alpha > 1.0){
        cerr << "alpha (" << alpha << ") > 1, assuming alpha = 1" << endl;
        alpha = 1.0;
    }
    StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);

    return x;
}
