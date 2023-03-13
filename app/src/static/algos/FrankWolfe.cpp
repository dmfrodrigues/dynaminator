#include "static/algos/FrankWolfe.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include "convex/GoldenSectionSolver.hpp"
#include "shortest-path/Dijkstra.hpp"

using namespace std;

typedef StaticNetwork::Node Node;

FrankWolfe::FrankWolfe(StaticProblem prob)
    : problem(prob) {}

void FrankWolfe::setStartingSolution(StaticSolution startingSolution) {
    xn = startingSolution;
}

void FrankWolfe::setStopCriteria(double e) {
    epsilon = e;
}

StaticSolution FrankWolfe::solve() {
    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.
    double prevCost = problem.supply.evaluate(xn);
    for (int it = 0; it < 100; ++it) {
        StaticSolution xstar = step1();
        xn = step2(xstar);

        double cost = problem.supply.evaluate(xn);

        double delta = prevCost - cost;

        cerr << "FW, it " << it << ", delta=" << delta << endl;

        if (delta < 0) {
            cerr << "FW: Cost increased. Stopping" << endl;
            return xn;
        } else if (delta < epsilon) {
            cerr << "FW: Cost did not improve more than " << epsilon << ". Stopping" << endl;
            return xn;
        }

        prevCost = cost;
    }

    return xn;
}

StaticSolution FrankWolfe::step1() {
    // TODO: duplicate code with AllOrNothing::solve() (AllOrNothing.cpp)
    StaticSolution xstar;

    Graph G = problem.supply.toGraph(xn);

    const vector<Node> startNodes = problem.demand.getStartNodes();
    for (const Node &u : startNodes) {
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());

        sp.get()->initialize(&G, u);
        sp.get()->run();

        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for (const Node &v : endNodes) {
            Graph::Path path = sp.get()->getPath(v);

            if (path.size() == 1 && path.front().id == Graph::EDGE_INVALID.id)
                throw logic_error("Could not find path " + to_string(u) + "->" + to_string(v));

            StaticNetwork::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for (const Graph::Edge &e : path)
                pathNetwork.push_back(e.id);

            double f = problem.demand.getDemand(u, v);
            xstar.addPath(pathNetwork, f);
        }
    }

    return xstar;
}

StaticSolution FrankWolfe::step2(const StaticSolution &xstar) {
    // TODO: allow to tune this value of epsilon
    const double EPSILON = 1e-4;
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
    ](double alpha) {
        StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);
        StaticNetwork::Cost c = problem.supply.evaluate(x);
        return c;
    };
    solver.get()->setProblem(p);

    double alpha = solver.get()->solve();
    StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);

    return x;
}
