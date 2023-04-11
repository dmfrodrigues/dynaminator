#pragma once

#include "Com/Message.hpp"
#include "GlobalState.hpp"

namespace Com {
class RunFWSimulation: public MessageRequest {
   private:
    std::string netPath, tazPath, demandPath;
    std::string edgeDataPath, routesPath;

   public:
    RunFWSimulation();
    RunFWSimulation(
        const std::string &netPath,
        const std::string &tazPath,
        const std::string &demandPath,
        const std::string &edgeDataPath,
        const std::string &routesPath
    );

    virtual Operation getOperation() const;
    virtual void      serializeContents(std::stringstream &ss) const;
    virtual bool      deserializeContents(std::stringstream &ss);

    class Response: public MessageResponse {
        virtual Operation getOperation() const;
        virtual void      serializeContents(std::stringstream &ss) const;
        virtual bool      deserializeContents(std::stringstream &ss);

        virtual void handle(std::ostream &is);
    };

    virtual Response *process();
};
}  // namespace Com
