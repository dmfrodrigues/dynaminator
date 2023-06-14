#include "Opt/GeneticSolver.hpp"

#include <cmath>
#include <iostream>
#include <random>

#include "Opt/UnivariateSolver.hpp"

using namespace std;
using namespace Opt;

typedef UnivariateSolver::Var Var;

GeneticSolver::GeneticSolver(
    size_t              populationSize_,
    size_t              newPopulationSize_,
    Var                 variabilityCoeff_,
    size_t              maxNumberGenerations_,
    int                 parallelism,
    shared_ptr<mt19937> gen_
):
    populationSize(populationSize_),
    newPopulationSize(newPopulationSize_),
    variabilityCoeff(variabilityCoeff_),
    maxNumberGenerations(maxNumberGenerations_),
    pool(parallelism),
    gen(gen_) {}

void GeneticSolver::clearInitialSolutions() {
    newPopulation.clear();
}

void GeneticSolver::addInitialSolution(Var v) {
    newPopulation.push_back(v);
}

void GeneticSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var GeneticSolver::solve(Problem p) {
    population.clear();

    problem = &p;

    tournament();

    for(size_t i = 0; i < maxNumberGenerations; ++i) {
        Var a = 1e9, b = -1e9;
        for(const auto &[z, v]: population) {
            a = min(a, v);
            b = max(b, v);
        }

        Var Delta = b - a;
        if(Delta < epsilon) {
            break;
        }

        crossover();
        mutation();
        tournament();
    }

    return population[0].second;
}

void GeneticSolver::crossover() {
    uniform_int_distribution<size_t> distribution(0, populationSize - 1);

    while(newPopulation.size() < newPopulationSize) {
        size_t a = distribution(*gen);
        size_t b = distribution(*gen);
        if(a == b) continue;

        newPopulation.push_back(crossover(population[a].second, population[b].second));
    }
}

Var GeneticSolver::crossover(const Var &a, const Var &b) {
    uniform_real_distribution<double> distribution(0.0, 1.0);

    double r = distribution(*gen);

    return a + (b - a) * r;
}

void GeneticSolver::mutation() {
    Var variability = getVariability();
    for(auto &v: newPopulation) {
        mutation(v, variability);
    }
}

Var &GeneticSolver::mutation(Var &v, const Var &variability) {
    uniform_real_distribution<double> distribution(-1.0, 1.0);

    double r = distribution(*gen);

    return v += r * variability;
}

Var GeneticSolver::getVariability() const {
    Var a = 1e9, b = -1e9;
    for(const auto &[z, v]: population) {
        a = min(a, v);
        b = max(b, v);
    }
    return (b - a) * variabilityCoeff;
}

void GeneticSolver::tournament() {
    if(pool.size() <= 0) {
        for(const Var &v: newPopulation)
            population.emplace_back((*problem)(v), v);
    } else {
        vector<future<Var>> results;
        for(const Var &v: newPopulation) {
            results.emplace_back(pool.push([this, v](int) -> Var {
                return (*problem)(v);
            }));
        }
        for(size_t i = 0; i < results.size(); ++i) {
            Var z = results[i].get();
            population.emplace_back(z, newPopulation[i]);
        }
    }

    newPopulation.clear();

    sort(population.begin(), population.end());

    while(population.size() > populationSize) {
        population.pop_back();
    }
}
