#pragma once

#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "Static/Demand.hpp"
#include "Static/SUMOAdapter.hpp"
#include "Static/supply/Network.hpp"
#include "utils/pipestream.hpp"
#include "utils/synchronizer.hpp"

struct GlobalState {
    typedef std::string ResourceID;

    class ResourceException: public std::runtime_error {
        const ResourceID id;

       public:
        ResourceException(const ResourceID &id, const std::string &msg);
        const ResourceID &getID() const { return id; }
    };

    class ResourceDoesNotExistException: public ResourceException {
       public:
        ResourceDoesNotExistException(const ResourceID &id);
    };

    class ResourceAlreadyExistsException: public ResourceException {
       public:
        ResourceAlreadyExistsException(const ResourceID &id);
    };

    struct TaskReturn {
        int         status;
        std::string content;
        std::string content_type = "text/plain";
    };

    // clang-format off
    class Streams: private utils::synchronizer<
        std::unordered_map<
            ResourceID,
            std::shared_ptr<utils::pipestream>
        >
    > {
        // clang-format on
       public:
        utils::pipestream &create(const ResourceID &id);
        utils::pipestream &get(const ResourceID &id);
        void               erase(const ResourceID &id);
    };

    static Streams streams;

    // clang-format off
    class Tasks: private utils::synchronizer<
        std::unordered_map<
            ResourceID,
            std::shared_ptr<std::shared_future<TaskReturn>>
        >
    > {
        // clang-format on
       public:
        std::shared_future<TaskReturn> &create(
            const ResourceID                  &id,
            const std::function<TaskReturn()> &f
        );
        std::shared_future<TaskReturn> &get(const ResourceID &id);
        void                            erase(const ResourceID &id);
    };

    static Tasks tasks;

   private:
    GlobalState();
};
