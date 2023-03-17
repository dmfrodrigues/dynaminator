#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "data/SumoAdapterStatic.hpp"
#include "static/StaticDemand.hpp"
#include "static/supply/StaticNetwork.hpp"

struct GlobalState {
    typedef std::string ResourceId;

   private:
    GlobalState();
};
