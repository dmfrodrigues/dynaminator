#pragma once

#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "Static/Demand.hpp"
#include "Static/supply/Network.hpp"
#include "data/SumoAdapterStatic.hpp"
#include "utils/pipestream.hpp"
#include "utils/synchronizer.hpp"

struct GlobalState {
    typedef std::string ResourceID;

    struct TaskReturn {
        int status;
        std::string content;
        std::string content_type = "text/plain";
    };

    static utils::synchronizer<
        std::unordered_map<
            ResourceID,
            std::shared_ptr<utils::pipestream>>>
        streams;

    static utils::synchronizer<
        std::unordered_map<
            ResourceID,
            std::shared_ptr<std::shared_future<TaskReturn>>>>
        tasks;

   private:
    GlobalState();
};
