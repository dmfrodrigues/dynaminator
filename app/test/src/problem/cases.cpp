#include "test/problem/cases.hpp"

#include "static/supply/CustomStaticNetwork.hpp"

#include <cmath>

using namespace std;

StaticNetwork *getStaticNetworkTestCase1(){
    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addNode(3);
    network->addEdge(1, 1, 2,
        [](StaticNetwork::Flow x){ return x*x + 2; },
        [](StaticNetwork::Flow x){ return x*x*x/3.0 + x*2.0; });
    network->addEdge(2, 1, 2,
        [](StaticNetwork::Flow x){ return 3*x + 1; },
        [](StaticNetwork::Flow x){ return (3.0/2.0) * x*x + x; });
    network->addEdge(3, 2, 3,
        [](StaticNetwork::Flow x){ return   x + 3; },
        [](StaticNetwork::Flow x){ return 0.5 * x * x + 3.0*x; });

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
    const double alpha = 0.15, beta = 4.0;
    const double c1 = 4400.0, c2 = 2200.0;
    const double t1 = 20.0, t2 = 10.0;

    CustomStaticNetwork *network = new CustomStaticNetwork();

    network->addNode(1);
    network->addNode(2);
    network->addEdge(1, 1, 2,
        [alpha, beta, t1, c1](double x){ return t1*(1.0 + alpha * pow(x/c1, beta)); },
        [alpha, beta, t1, c1](double x){ return t1*x*((alpha/(beta+1)) * pow(x/c1, beta) + 1); });
    network->addEdge(2, 1, 2,
        [alpha, beta, t2, c2](double x){ return t2*(1.0 + alpha * pow(x/c2, beta)); },
        [alpha, beta, t2, c2](double x){ return t2*x*((alpha/(beta+1)) * pow(x/c2, beta) + 1); });

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

    const double alpha = 0.15, beta = 4.0;
    const double t1 = 20.0, t2 = 10.0;
    const double c1 = 4400.0, c2 = 2200.0;

    network->addNode(1);
    network->addNode(2);
    network->addEdge(1, 1, 2,
        // [alpha, beta, t1, c1](double x){ return t1*(1.0 + alpha * (x/c1 + beta) * pow(x/c1, beta-1.0)); },
        [alpha, beta, t1, c1](double x){ return t1*(1.0 + alpha * pow(x/c1, beta)); },
        [alpha, beta, t1, c1](double x){ return x*t1*(1.0 + alpha * pow(x/c1, beta)); }
    );
    network->addEdge(2, 1, 2,
        // [alpha, beta, t2, c2](double x){ return t2*(1.0 + alpha * (x/c2 + beta) * pow(x/c2, beta-1.0)); },
        [alpha, beta, t2, c2](double x){ return t2*(1.0 + alpha * pow(x/c2, beta)); },
        [alpha, beta, t2, c2](double x){ return x*t2*(1.0 + alpha * pow(x/c2, beta)); }
    );

    return network;
}

StaticProblem *getStaticProblemTestCase3(){
    StaticNetwork *network = getStaticNetworkTestCase3();

    StaticDemand demand;
    demand.addDemand(1, 2, 7000);

    StaticProblem *problem = new StaticProblem{*network, demand};
    return problem;
}

