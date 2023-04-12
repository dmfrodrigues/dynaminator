#include "GlobalState.hpp"

using namespace std;

utils::synchronizer<
    unordered_map<
        GlobalState::ResourceID,
        shared_ptr<stringstream> > >
    GlobalState::streams;
