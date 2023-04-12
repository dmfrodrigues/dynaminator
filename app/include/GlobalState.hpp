#pragma once

#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <memory>

#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"
#include "data/SumoAdapterStatic.hpp"

struct GlobalState {
    typedef std::string ResourceId;

    std::unordered_map<ResourceId, std::shared_ptr<std::stringstream>> streams;

   private:
    GlobalState();
};
