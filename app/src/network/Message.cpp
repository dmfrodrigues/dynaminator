#include "Com/Message.hpp"

#include "utils/serialize.hpp"

using namespace std;
using namespace Com;

typedef Message::Type Type;

std::unordered_map<
    Message::Operation,
    std::function<Message *()> >
    Message::generators;

void Message::registerOperation(
    const Operation &operation,
    std::function<Message *()> generator) {
    generators[operation] = generator;
}

stringstream Message::serialize() const {
    stringstream ss;

    ss << utils::serialize<Operation>(getOperation());

    serializeContents(ss);
    return ss;
}

utils::serialize<Type>::serialize(const Type &obj):t(obj){}
ostream &std::operator<<(ostream &os, const utils::serialize<Type> &s){
    os.write(reinterpret_cast<const char*>(&s.t), sizeof(s.t));
    return os;
}

utils::deserialize<Type>::deserialize(Type &obj): t(obj){}
istream &std::operator>>(istream &is, utils::deserialize<Type> s) {
    is.read(reinterpret_cast<char*>(&s.t), sizeof(s.t));
    return is;
}

MessageRequest::Type MessageRequest::getType() const {
    return REQUEST;
}

void MessageResponse::setStatusCode(int status){
    statusCode = status;
}

int MessageResponse::getStatusCode() const {
    return statusCode;
}

void MessageResponse::setReason(const string &s) {
    reason = s;
}
const string &MessageResponse::getReason() const {
    return reason;
}

MessageResponse::Type MessageResponse::getType() const {
    return RESPONSE;
}
