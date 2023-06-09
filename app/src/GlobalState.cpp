#include "GlobalState.hpp"
#include <stdexcept>

using namespace std;

GlobalState::ResourceException::ResourceException(
    const GlobalState::ResourceID &id_,
    const string &msg_
):
    runtime_error(msg_),
    id(id_)
{}

GlobalState::ResourceDoesNotExistException::ResourceDoesNotExistException(
    const GlobalState::ResourceID &id_
):
    ResourceException(id_, "Resource " + id_ + " does not exist")
{}

GlobalState::ResourceAlreadyExistsException::ResourceAlreadyExistsException(
    const GlobalState::ResourceID &id_
):
    ResourceException(id_, "Resource " + id_ + " already exists")
{}

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

GlobalState::Streams GlobalState::streams;

utils::synchronizer<
    unordered_map<
        GlobalState::ResourceID,
        shared_ptr<shared_future<GlobalState::TaskReturn>>>>
    GlobalState::tasks;
