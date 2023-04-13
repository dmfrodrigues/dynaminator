#pragma once

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"
#include "data/SumoAdapterStatic.hpp"
#include "utils/pipestream.hpp"
#include "utils/synchronizer.hpp"

struct GlobalState {
    typedef std::string ResourceID;

    static utils::synchronizer<
        std::unordered_map<
            ResourceID,
            std::shared_ptr<utils::pipestream> > >
        streams;

   private:
    GlobalState();
};
