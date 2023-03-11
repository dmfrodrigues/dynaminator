#pragma once

#include <string>
#include <unordered_map>

#include "static/supply/StaticNetwork.hpp"

struct GlobalState {
    typedef std::string ResourceId;

    static std::unordered_map<ResourceId, StaticNetwork*> staticNetworks;

private:
    GlobalState();
};
