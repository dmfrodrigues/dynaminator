#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include "data/SUMO/Additionals/Location.hpp"

using namespace std;
using namespace rapidxml;
using namespace SUMO::Additionals;
using namespace utils::stringify;

SUMO::Coord Location::center() const {
    return Coord(
        (convBoundary.first.X + convBoundary.second.X) / 2,
        (convBoundary.first.Y + convBoundary.second.Y) / 2
    );
}

SUMO::Coord Location::size() const {
    return Coord(
        convBoundary.second.X - convBoundary.first.X,
        convBoundary.second.Y - convBoundary.first.Y
    );
}

void Location::loadFromXMLNode(const rapidxml::xml_node<> &node) {
    xml_attribute<> *netOffsetAttr = node.first_attribute("netOffset");
    assert(netOffsetAttr != nullptr);
    netOffset = stringify<SUMO::Coord>::fromString(netOffsetAttr->value());

    xml_attribute<> *convBoundaryAttr = node.first_attribute("convBoundary");
    assert(convBoundaryAttr != nullptr);
    convBoundary = stringify<pair<Coord, Coord>>::fromString(convBoundaryAttr->value());

    xml_attribute<> *origBoundaryAttr = node.first_attribute("origBoundary");
    assert(origBoundaryAttr != nullptr);
    origBoundary = stringify<pair<Coord, Coord>>::fromString(origBoundaryAttr->value());

    xml_attribute<> *projParameterAttr = node.first_attribute("projParameter");
    assert(projParameterAttr != nullptr);
    projParameter = projParameterAttr->value();
}
