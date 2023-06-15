#include "Dynamic/Env/Lane.hpp"

#include "Dynamic/Env/Edge.hpp"

using namespace std;
using namespace Dynamic;
using namespace Dynamic::Env;

Lane::Lane(const Edge &edge, Index index):
    edge(edge), index(index) {}

const Lane Lane::INVALID = {Edge::INVALID, 0};
