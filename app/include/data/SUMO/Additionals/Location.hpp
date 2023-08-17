#pragma once

#include "data/SUMO/Additionals/Additionals.hpp"

namespace SUMO::Additionals {

class Location: public Additional {
   public:
    SUMO::Coord             netOffset;
    std::pair<Coord, Coord> convBoundary;
    std::pair<Coord, Coord> origBoundary;
    std::string             projParameter;

    Coord center() const;
    Coord size() const;

    virtual void loadFromXMLNode(const rapidxml::xml_node<>& node) override;
};

}  // namespace SUMO::Additionals
