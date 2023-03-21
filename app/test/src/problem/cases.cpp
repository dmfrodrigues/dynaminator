#include "test/problem/cases.hpp"

#include "static/supply/CustomStaticNetwork.hpp"

#include <cmath>

using namespace std;

StaticNetwork *getStaticNetworkTestCase1(){
    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addNode(3);
    network->addEdge(1, 1, 2, [](StaticNetwork::Flow x){ return x*x + 2; });
    network->addEdge(2, 1, 2, [](StaticNetwork::Flow x){ return 3*x + 1; });
    network->addEdge(3, 2, 3, [](StaticNetwork::Flow x){ return   x + 3; });

    return network;
}

StaticProblem *getStaticProblemTestCase1(){
    StaticNetwork *network = getStaticNetworkTestCase1();

    StaticDemand demand;
    demand.addDemand(1, 3, 4);

    StaticProblem *problem = new StaticProblem{*network, demand};
    return problem;
}

StaticNetwork *getStaticNetworkTestCase2(){
    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addEdge(1, 1, 2, [](double x){ return 20.0*(1.0 + 0.15 * pow(x/4400.0, 4.0)); });
    network->addEdge(2, 1, 2, [](double x){ return 10.0*(1.0 + 0.15 * pow(x/2200.0, 4.0)); });

    return network;
}

StaticProblem *getStaticProblemTestCase2(){
    StaticNetwork *network = getStaticNetworkTestCase2();

    StaticDemand demand;
    demand.addDemand(1, 2, 7000);

    StaticProblem *problem = new StaticProblem{*network, demand};
    return problem;
}

StaticNetwork *getStaticNetworkTestCase3(){
    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addEdge(1, 1, 2, [](double x){ return 20.0*((0.15)/(4.0 + 1.0) * pow(x/4400.0, 4.0) + 1.0); });
    network->addEdge(2, 1, 2, [](double x){ return 10.0*((0.15)/(4.0 + 1.0) * pow(x/2200.0, 4.0) + 1.0); });

    return network;
}

StaticProblem *getStaticProblemTestCase3(){
    StaticNetwork *network = getStaticNetworkTestCase3();

    StaticDemand demand;
    demand.addDemand(1, 2, 7000);

    StaticProblem *problem = new StaticProblem{*network, demand};
    return problem;
}

