#include "GlobalState.hpp"

using namespace std;

unordered_map<GlobalState::ResourceId, pair<StaticNetwork*, SumoAdapterStatic>> GlobalState::staticNetworks;
