#pragma once

#include <functional>
#include <sstream>
#include <unordered_map>

#include "utils/serialize.hpp"

namespace Com {
class MessageRequest;
class MessageResponse;

class MessageFactory;

class Message {
    friend MessageFactory;

    friend MessageRequest;
    friend MessageResponse;

   public:
    typedef std::string Operation;

   protected:
    static std::unordered_map<Operation, std::function<Message *()>> generators;

   public:
    enum Type : uint8_t {
        REQUEST  = 1,
        RESPONSE = 2
    };

    virtual Type      getType() const      = 0;
    virtual Operation getOperation() const = 0;

   protected:
    virtual void serializeContents(std::stringstream &ss) const = 0;
    virtual bool deserializeContents(std::stringstream &ss)     = 0;

   public:
    std::stringstream serialize() const;

    static void registerOperation(
        const Operation           &operation,
        std::function<Message *()> generator
    );

    virtual ~Message() {}
};

class MessageRequest: public Message {
   private:
    virtual Type getType() const;

   public:
    virtual MessageResponse *process() = 0;
};

class MessageResponse: public Message {
   private:
    int         statusCode = 200;
    std::string reason     = "";

    virtual Type getType() const;

   public:
    virtual void setStatusCode(int status);
    virtual int  getStatusCode() const;

    virtual void               setReason(const std::string &s);
    virtual const std::string &getReason() const;

    virtual void handle(std::ostream &is) = 0;
};
}  // namespace Com

namespace std {
ostream &operator<<(ostream &os, const utils::serialize::serialize<Com::Message::Type> &s);
istream &operator>>(istream &os, utils::serialize::deserialize<Com::Message::Type> s);
}  // namespace std

namespace utils::serialize {
template<>
class serialize<Com::Message::Type> {
    const Com::Message::Type &t;

   public:
    serialize(const Com::Message::Type &obj);
    friend std::ostream &std::operator<<(
        std::ostream &os, const serialize<Com::Message::Type> &s
    );
};

template<>
class deserialize<Com::Message::Type> {
    Com::Message::Type &t;

   public:
    deserialize(Com::Message::Type &obj);
    friend std::istream &std::operator>>(
        std::istream                   &is,
        deserialize<Com::Message::Type> s
    );
};
}  // namespace utils::serialize

#define MESSAGE_REGISTER_MAIN(MessageSubclass)               \
    Com::Message::registerOperation(#MessageSubclass, []() { \
        return new MessageSubclass();                        \
    })

#define MESSAGE_REGISTER_DEF(MessageSubclass)                       \
    Com::Message::Operation MessageSubclass::getOperation() const { \
        return #MessageSubclass;                                    \
    }
