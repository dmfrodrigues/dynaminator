#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "data/SumoAdapterStatic.hpp"
#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"

struct GlobalState {
    typedef std::string ResourceId;

   private:
    GlobalState();
};
