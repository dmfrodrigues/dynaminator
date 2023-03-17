#pragma once

#include "GlobalState.hpp"
#include "network/Message.hpp"

class RunFWSimulation : public MessageRequest {
   private:
    std::string netPath, tazPath, demandPath;
    std::string outPath;

   public:
    RunFWSimulation();
    RunFWSimulation(
        const std::string &netPath,
        const std::string &tazPath,
        const std::string &demandPath,
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
