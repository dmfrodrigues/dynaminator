#include "GlobalState.hpp"

using namespace std;

utils::synchronizer<
    unordered_map<
        GlobalState::ResourceID,
        shared_ptr<utils::pipestream> > >
    GlobalState::streams;
