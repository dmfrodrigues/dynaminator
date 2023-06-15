#pragma once

#include <memory>
#include <string>

#include "Dynamic/Dynamic.hpp"

namespace Dynamic {

namespace Env {
class Env;
class Edge;
class Connection;
}  // namespace Env

class Vehicle {
   public:
    typedef long ID;

    class Policy {
       public:
        class Factory {
           public:
            virtual std::shared_ptr<Policy> create(
                Vehicle::ID      id,
                Time             depart,
                const Env::Edge &from,
                const Env::Edge &to
            ) = 0;
        };

        virtual const Env::Connection &pickConnection(
            const Env::Env &env
        ) = 0;
    };

    const ID         id;
    const Time       depart;
    const Env::Edge &from, &to;

   protected:
    std::shared_ptr<Policy> policy;

   public:
    Vehicle(
        ID                      id,
        Time                    depart,
        const Env::Edge        &from,
        const Env::Edge        &to,
        std::shared_ptr<Policy> policy
    );
};
}  // namespace Dynamic
