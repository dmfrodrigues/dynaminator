#include "GlobalState.hpp"

using namespace std;

utils::synchronizer<
    unordered_map<
        GlobalState::ResourceID,
        shared_ptr<utils::pipestream> > >
    GlobalState::streams;

utils::synchronizer<
    unordered_map<
        GlobalState::ResourceID,
        shared_ptr<shared_future<GlobalState::TaskReturn>>>>
    GlobalState::tasks;
