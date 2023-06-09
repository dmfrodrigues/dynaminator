#include "GlobalState.hpp"

#include <exception>
#include <functional>
#include <stdexcept>

using namespace std;

GlobalState::ResourceException::ResourceException(
    const GlobalState::ResourceID &id_,
    const string                  &msg_
):
    runtime_error(msg_),
    id(id_) {}

GlobalState::ResourceDoesNotExistException::ResourceDoesNotExistException(
    const GlobalState::ResourceID &id_
):
    ResourceException(id_, "Resource " + id_ + " does not exist") {}

GlobalState::ResourceAlreadyExistsException::ResourceAlreadyExistsException(
    const GlobalState::ResourceID &id_
):
    ResourceException(id_, "Resource " + id_ + " already exists") {}

utils::pipestream &GlobalState::Streams::create(const GlobalState::ResourceID &id) {
    lock_guard<mutex> lock(*this);

    auto [it, success] = GlobalState::streams->emplace(id, make_shared<utils::pipestream>());
    if(!success) {
        throw ResourceAlreadyExistsException(id);
    }
    return *it->second.get();
}

utils::pipestream &GlobalState::Streams::get(const GlobalState::ResourceID &id) {
    lock_guard<mutex> lock(*this);

    auto it = (*this)->find(id);
    if(it == streams->end()) {
        throw ResourceDoesNotExistException(id);
    }
    return *it->second.get();
}

void GlobalState::Streams::erase(const GlobalState::ResourceID &id) {
    lock_guard<mutex> lock(*this);

    (*this)->erase(id);
}

shared_future<GlobalState::TaskReturn> &GlobalState::Tasks::create(
    const GlobalState::ResourceID &id,
    const function<TaskReturn()>  &f
) {
    lock_guard<mutex> lock(*this);

    auto [it, success] = GlobalState::tasks->emplace(
        id,
        make_shared<shared_future<GlobalState::TaskReturn>>()
    );

    if(!success) {
        throw ResourceAlreadyExistsException(id);
    }

    shared_future<TaskReturn> &future = *(it->second);

    future = shared_future<TaskReturn>(
        async(launch::async, [this, id, f]() -> TaskReturn {
            try {
                TaskReturn ret = f();

                lock_guard<mutex> taskLock(*this);

                thread([this, id]() {
                    this->erase(id);
                }).detach();

                return ret;
            } catch(const exception &e) {
                return {500, "what(): "s + e.what()};
            }
        })
    );

    return future;
}

shared_future<GlobalState::TaskReturn> &GlobalState::Tasks::get(
    const GlobalState::ResourceID &id
) {
    lock_guard<mutex> lock(*this);

    auto it = (*this)->find(id);
    if(it == tasks->end()) {
        throw ResourceDoesNotExistException(id);
    }
    return *it->second.get();
}

void GlobalState::Tasks::erase(const GlobalState::ResourceID &id) {
    lock_guard<mutex> lock(*this);

    auto it = (*this)->find(id);

    const auto &[_, future] = *it;

    future->wait();

    (*this)->erase(it);
}

GlobalState::Streams GlobalState::streams;

GlobalState::Tasks GlobalState::tasks;
