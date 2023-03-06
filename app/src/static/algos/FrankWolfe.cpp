#include "static/algos/FrankWolfe.hpp"

#include "shortest-path/Dijkstra.hpp"
#include "convex/GoldenSectionSolver.hpp"

#include <memory>
#include <utility>

#include <iostream>

using namespace std;

typedef StaticNetwork::Node Node;

FrankWolfe::FrankWolfe(StaticProblem prob)
    :problem(prob)
{}

void FrankWolfe::setStartingSolution(StaticSolution startingSolution){
    xn = startingSolution;
}

StaticSolution FrankWolfe::solve(){
    // TODO: allow to change number of iterations.
    // TODO: consider using epsilon instead of number of iterations to decide when to stop.
    for(int it = 0; it < 1000; ++it){
        StaticSolution xstar = step1();
        xn = step2(xstar);
    }

    return xn;
}

StaticSolution FrankWolfe::step1(){
    // TODO: duplicate code with AllOrNothing::solve() (AllOrNothing.cpp)
    StaticSolution xstar;

    Graph G = problem.supply.toGraph(xn);

    const vector<Node> startNodes = problem.demand.getStartNodes();
    for(const Node &u: startNodes){
        unique_ptr<ShortestPathOneMany> sp(new Dijkstra());

        sp.get()->initialize(&G, u);
        sp.get()->run();

        const vector<Node> endNodes = problem.demand.getDestinations(u);
        for(const Node &v: endNodes){
            Graph::Path path = sp.get()->getPath(v);
    
            StaticNetwork::Path pathNetwork;
            pathNetwork.reserve(path.size());
            for(const Graph::Edge &e: path)
                pathNetwork.push_back(e.id);

            double f = problem.demand.getDemand(u, v);
            xstar.addPath(pathNetwork, f);
        }
    }

    return xstar;
}

StaticSolution FrankWolfe::step2(const StaticSolution &xstar){
    // TODO: allow to tune this value of epsilon
    const double EPSILON = 1e-6;
    unique_ptr<ConvexSolver> solver; {
        IntervalSolver *is = new GoldenSectionSolver();
        is->setInterval(0, 1);
        is->setStopCriteria(EPSILON);

        solver = unique_ptr<ConvexSolver>(is);
    }
    ConvexSolver::Problem p = [
        &problem = as_const(problem),
        &xn = as_const(xn),
        &xstar = as_const(xstar)
    ](double alpha){
        StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);
        StaticNetwork::Cost c = problem.supply.evaluate(x);
        return c;
    };
    solver.get()->setProblem(
        p
    );

    double alpha = solver.get()->solve();
    StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);

    return x;
}
