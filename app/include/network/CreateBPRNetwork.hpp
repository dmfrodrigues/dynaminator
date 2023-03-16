#pragma once

#include "GlobalState.hpp"
#include "network/Message.hpp"

class CreateBPRNetwork : public MessageRequest {
   private:
    GlobalState::ResourceId resourceId;
    std::string netPath, tazPath;

   public:
    CreateBPRNetwork();
    CreateBPRNetwork(
        const GlobalState::ResourceId &resourceId,
        const std::string &netPath,
        const std::string &tazPath
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
