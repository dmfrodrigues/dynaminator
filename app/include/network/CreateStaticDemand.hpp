#pragma once

#include "GlobalState.hpp"
#include "network/Message.hpp"

class CreateStaticDemand : public MessageRequest {
   private:
    GlobalState::ResourceId resourceId;
    GlobalState::ResourceId networkId;
    std::string path;

   public:
    CreateStaticDemand();
    CreateStaticDemand(
        const GlobalState::ResourceId &resourceId,
        const GlobalState::ResourceId &networkId,
        const std::string &path
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
