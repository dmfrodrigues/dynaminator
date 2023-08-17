#pragma once

#include <color/color.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <rapidxml.hpp>
#include <string>

#include "data/SUMO/SUMO.hpp"

namespace SUMO::Additionals {

class Additional {
   public:
    virtual void loadFromXMLNode(const rapidxml::xml_node<>& node) = 0;
};

std::vector<std::unique_ptr<Additional>> loadFromFile(const std::filesystem::path& path);

class XMLFactory {
   public:
    std::unique_ptr<Additional> create(const rapidxml::xml_node<>& node);
};

}  // namespace SUMO::Additionals
