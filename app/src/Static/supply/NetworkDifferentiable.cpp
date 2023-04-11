#include "Static/supply/NetworkDifferentiable.hpp"

using namespace Static;

NetworkDifferentiable::Edge::Edge(Edge::ID id_, Node u_, Node v_):
    Network::Edge(id_, u_, v_) {}
