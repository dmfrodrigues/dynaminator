#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "data/SumoAdapterStatic.hpp"
#include "static/supply/StaticNetwork.hpp"

struct GlobalState {
    typedef std::string ResourceId;

    static std::unordered_map<ResourceId, std::pair<StaticNetwork*, SumoAdapterStatic>> staticNetworks;

   private:
    GlobalState();
};
