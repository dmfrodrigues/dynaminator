#include "network/Message.hpp"

using namespace std;

std::unordered_map<
    Message::Operation,
    std::function<Message *()> >
    Message::generators;

void Message::registerOperation(
    const Operation &operation,
    std::function<Message *()> generator) {
    generators[operation] = generator;
}

MessageRequest::Type MessageRequest::getType() const {
    return REQUEST;
}

MessageResponse::Type MessageResponse::getType() const {
    return RESPONSE;
}
