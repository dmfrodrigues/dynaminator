#include "utils/serialize.hpp"

using namespace std;

utils::serialize<string>::serialize(const string &obj)
: t(obj){}

ostream &std::operator<<(ostream &os, const utils::serialize<string> &s) {
    size_t sz = s.t.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(sz));
    os.write(s.t.data(), sz);
    return os;
}

utils::deserialize<string>::deserialize(string &obj)
: t(obj){}

istream &std::operator>>(istream &is, utils::deserialize<string> s) {
    size_t sz = 0;
    is.read(reinterpret_cast<char*>(&sz), sizeof(sz));
    char *buf = new char[sz+1];
    is.read(buf, sz);
    buf[sz] = '\0';
    s.t = string(buf);
    delete buf;
    return is;
}

utils::serialize<bool>::serialize(bool obj)
: t(obj){}

ostream &std::operator<<(ostream &os, const utils::serialize<bool> &s) {
    os.write(reinterpret_cast<const char*>(&s.t), sizeof(s.t));
    return os;
}

utils::deserialize<bool>::deserialize(bool &obj)
: t(obj){}

istream &std::operator>>(istream &is, utils::deserialize<bool> s) {
    is.read(reinterpret_cast<char*>(&s.t), sizeof(s.t));
    return is;
}
