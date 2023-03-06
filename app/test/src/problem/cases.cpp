#include "test/problem/cases.hpp"

#include "static/supply/CustomStaticNetwork.hpp"

using namespace std;

unique_ptr<StaticNetwork> getStaticNetworkTestCase1(){
    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addNode(3);
    network->addEdge(1, 1, 2, [](double x){ return x*x + 2; });
    network->addEdge(2, 1, 2, [](double x){ return 3*x + 1; });
    network->addEdge(3, 2, 3, [](double x){ return   x + 3; });

    return unique_ptr<StaticNetwork>(network);
}

unique_ptr<StaticProblem> getStaticProblemTestCase1(){
    unique_ptr<StaticNetwork> network = getStaticNetworkTestCase1();

    StaticDemand demand;
    demand.addDemand(1, 3, 4);

    StaticProblem *problem = new StaticProblem{*network.get(), demand};
    return unique_ptr<StaticProblem>(problem);
}
