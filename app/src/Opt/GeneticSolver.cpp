#include "Opt/GeneticSolver.hpp"

#include <cmath>
#include <iostream>

#include "Opt/UnivariateSolver.hpp"

using namespace std;
using namespace Opt;

typedef UnivariateSolver::Var Var;

GeneticSolver::GeneticSolver(
    size_t populationSize_,
    Var    variabilityCoeff_,
    size_t tournamentSize_,
    size_t maxNumberGenerations_
):
    populationSize(populationSize_),
    variabilityCoeff(variabilityCoeff_),
    tournamentSize(tournamentSize_),
    maxNumberGenerations(maxNumberGenerations_) {}

void GeneticSolver::clearInitialSolutions() {
    population.clear();
}

void GeneticSolver::addInitialSolution(Var v) {
    population.push_back(make_pair(0, v));
}

void GeneticSolver::setStopCriteria(Var e) {
    epsilon = e;
}

Var GeneticSolver::solve(Problem p) {
    problem = &p;

    for(size_t i = 0; i < maxNumberGenerations; ++i) {
        Var a = 1e9, b = -1e9;
        for(const auto &[z, v]: population) {
            a = min(a, v);
            b = max(b, v);
        }
        if(b - a < epsilon) {
            break;
        }

        crossover();
        mutation();
        tournament();
    }

    return population[0].second;
}

void GeneticSolver::crossover() {
    vector<pair<Var, Var>> newPopulation;

    while(newPopulation.size() < tournamentSize){
        size_t a = rand() % populationSize;
        size_t b = rand() % populationSize;

        newPopulation.push_back(make_pair(
            0,
            crossover(population[a].second, population[b].second)
        ));
    }

    population = newPopulation;
}

Var GeneticSolver::crossover(const Var &a, const Var &b) {
    double r = (double)rand() / RAND_MAX;
    return a + (b - a) * r;
}

void GeneticSolver::mutation() {
    for(auto &[z, v]: population) {
        mutation(v);
    }
}

Var &GeneticSolver::mutation(Var &v) {
    double r = (double)rand() / RAND_MAX;

    r = r * 2 - 1;

    return v += r * getVariability();
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
    for(auto &[z, v]: population){
        z = (*problem)(v);
    }

    sort(population.begin(), population.end());

    while(population.size() > populationSize) {
        population.pop_back();
    }
}
