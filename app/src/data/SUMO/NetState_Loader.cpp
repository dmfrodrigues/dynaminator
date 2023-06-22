#include <iostream>

#include "Dynamic/Env/Env.hpp"
#include "Dynamic/Env/Lane.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/Network.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace SUMO;

using namespace utils::stringify;

// clang-format off
NetState::Timestep NetState::Timestep::Loader<
    Dynamic::Env::Env &,
    const Dynamic::SUMOAdapter &,
    Dynamic::Time
>::load(
    Dynamic::Env::Env &env,
    const Dynamic::SUMOAdapter &adapter,
    Dynamic::Time t
) {
    // clang-format on

    env.updateAllVehicles(t);

    Timestep ret{t};

    for(const Dynamic::Env::Vehicle &vehicle: env.getVehicles()) {
        if(vehicle.state == Dynamic::Env::Vehicle::State::LEFT)
            continue;

        const SUMO::Network::Edge::ID sumoEdgeID = adapter.toSumoEdge(vehicle.position.lane.edge.id);

        const SUMO::Length pos   = vehicle.position.offset;
        const SUMO::Speed  speed = vehicle.speed;

        if(!ret.edges.count(sumoEdgeID)) {
            ret.edges[sumoEdgeID] = Edge{sumoEdgeID};
        }

        Timestep::Edge &edge = ret.edges[sumoEdgeID];

        SUMO::Network::Edge::Lane::ID sumoLaneID = sumoEdgeID + "_" + to_string(vehicle.position.lane.index);
        if(!edge.lanes.count(sumoLaneID)) {
            edge.lanes[sumoLaneID] = Edge::Lane{sumoLaneID};
        }

        Timestep::Edge::Lane &lane = edge.lanes.begin()->second;

        lane.addVehicle(
            stringify<Dynamic::Env::Vehicle::ID>::toString(vehicle.id),
            pos,
            speed
        );
    }

    return ret;
}
