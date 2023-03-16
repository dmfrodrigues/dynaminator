#pragma once

#include "GlobalState.hpp"
#include "network/Message.hpp"

class RunFWSimulation : public MessageRequest {
   private:
    GlobalState::ResourceId networkId;
    GlobalState::ResourceId demandId;
    std::string outPath;

   public:
    RunFWSimulation();
    RunFWSimulation(
        const GlobalState::ResourceId &networkId,
        const GlobalState::ResourceId &demandId,
        const std::string &outPath
    );

    virtual Operation getOperation() const;
    virtual void serializeContents(std::stringstream &ss) const;
    virtual bool deserializeContents(std::stringstream &ss);

    class Response : public MessageResponse {
        virtual Operation getOperation() const;
        virtual void serializeContents(std::stringstream &ss) const;
        virtual bool deserializeContents(std::stringstream &ss);

        virtual void handle(std::ostream &is);
    };

    virtual Response *process();
};
