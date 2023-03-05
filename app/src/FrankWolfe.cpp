#include "FrankWolfe.hpp"

#include "shortest-path/Dijkstra.hpp"
#include "convex/GoldenSectionSolver.hpp"

#include <memory>
#include <utility>

using namespace std;

typedef StaticNetwork::Node Node;

void FrankWolfe::setStartingSolution(StaticSolution startingSolution){
    xn = startingSolution;
}

StaticSolution FrankWolfe::solve(){
    for(int it = 0; it < 1000; ++it){
        StaticSolution xstar = step1();
        xn = step2(xstar);
    }

    return xn;
}

StaticSolution FrankWolfe::step1(){
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
    const double EPSILON = 1e3;
    unique_ptr<ConvexSolver> solver; {
        IntervalSolver *is = new GoldenSectionSolver();
        is->setInterval(0, 1);
        is->setStopCriteria(EPSILON);

        solver = unique_ptr<ConvexSolver>(is);
    }
    solver.get()->setProblem(
        [
            &problem = as_const(problem),
            &xn = as_const(xn),
            &xstar = as_const(xstar)
        ](double alpha){
            StaticSolution x = StaticSolution::interpolate(xn, xstar, alpha);
            StaticNetwork::Cost c = problem.supply.evaluate(x);
            return c;
        }
    );

    double alpha = solver.get()->solve();
    return StaticSolution::interpolate(xn, xstar, alpha);
}
