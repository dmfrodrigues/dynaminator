#include "static/algos/FrankWolfe.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

#include "convex/GoldenSectionSolver.hpp"
#include "shortest-path/Dijkstra.hpp"

using namespace std;

typedef StaticNetwork::Node Node;
typedef StaticNetwork::Flow Flow;
typedef StaticNetwork::Cost Cost;

FrankWolfe::FrankWolfe(StaticProblem prob)
    : problem(prob) {}

void FrankWolfe::setStartingSolution(const StaticSolution &startingSolution) {
    xn = startingSolution;
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
    Flow prevCost = problem.supply.evaluate(xn);
    for (int it = 0; it < iterations; ++it) {
        StaticSolutionBase xstar = step1();

        xn = step2(xstar);

        Cost cost = problem.supply.evaluate(xn);

        Cost delta = prevCost - cost;

        cout << "FW, it " << it << ", delta=" << delta << ", cost=" << cost << endl;

        if (delta < 0) {
            cout << "FW: Cost increased. Stopping" << endl;
            return xn;
        } else if (delta < epsilon) {
            cout << "FW: Cost did not improve more than " << epsilon << ". Stopping" << endl;
            return xn;
        }

        prevCost = cost;
    }

    return xn;
}

StaticSolutionBase FrankWolfe::step1() {
    // TODO: duplicate code with AllOrNothing::solve() (AllOrNothing.cpp)
    StaticSolutionBase xstar;

    Graph G = problem.supply.toGraph(xn);

    unordered_map<Node, unique_ptr<ShortestPathOneMany>> shortestPaths;

    const vector<Node> startNodes = problem.demand.getStartNodes();
    for (const Node &u : startNodes) {
        shortestPaths.emplace(u, new Dijkstra());
        shortestPaths[u].get()->initialize(&G, u);
    }

    vector<future<void>> results;
    for (const Node &u : startNodes) {
        results.emplace_back(pool.push([&shortestPaths, u](int) {
            ShortestPathOneMany *sp = shortestPaths.at(u).get();
            sp->run();
        }));
    }
    for (future<void> &r : results) r.get();

    for (const Node &u : startNodes) {
        const ShortestPathOneMany *sp = shortestPaths[u].get();

        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for (const Node &v : endNodes) {
            Graph::Path path = sp->getPath(v);

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

    return xstar;
}

StaticSolution FrankWolfe::step2(const StaticSolution &xstar) {
    // TODO: allow to tune this value of epsilon
    const ConvexSolver::Var EPSILON = 1e-6;
    unique_ptr<ConvexSolver> solver;
    {
        IntervalSolver *is = new GoldenSectionSolver();
        is->setInterval(0, 1);
        is->setStopCriteria(EPSILON);

        solver = unique_ptr<ConvexSolver>(is);
    }
    ConvexSolver::Problem p = [
        &problem = as_const(problem),
        &xn = as_const(xn),
        &xstar = as_const(xstar)
    ](ConvexSolver::Var alpha) {
        StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);
        StaticNetwork::Cost c = problem.supply.evaluate(x);
        return c;
    };
    solver.get()->setProblem(p);

    ConvexSolver::Var alpha = solver.get()->solve();
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
