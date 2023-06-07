#include <iostream>
#include "Dynamic/Environment.hpp"
#include "data/SUMO/NetState.hpp"
#include "data/SUMO/SUMO.hpp"

using namespace std;
using namespace SUMO;

using namespace utils::stringify;

// clang-format off
NetState::Timestep NetState::Timestep::Loader<
    Dynamic::Environment &,
    const Dynamic::SUMOAdapter &,
    Dynamic::Time
>::load(
    Dynamic::Environment &env,
    const Dynamic::SUMOAdapter &adapter,
    Dynamic::Time t
) {
    // clang-format on

    env.updateAllVehicles(t);

    Timestep ret{t};

    for(const auto &[vehicleID, vehicle]: env.getVehicles()) {
        const SUMO::Network::Edge::ID sumoEdgeID = adapter.toSumoEdge(vehicle.position.edge);

        const SUMO::Length pos   = vehicle.position.offset;
        const SUMO::Speed  speed = vehicle.speed;

        if(!ret.edges.count(sumoEdgeID)){
            ret.edges[sumoEdgeID] = Edge{sumoEdgeID};
            ret.edges[sumoEdgeID].addLane(sumoEdgeID + "_0");
        }
        Timestep::Edge &edge = ret.edges[sumoEdgeID];
        Timestep::Edge::Lane &lane = edge.lanes.begin()->second;

        lane.addVehicle(
            stringify<Dynamic::Environment::Vehicle::ID>::toString(vehicle.id),
            pos,
            speed
        );
    }

    return ret;
}
