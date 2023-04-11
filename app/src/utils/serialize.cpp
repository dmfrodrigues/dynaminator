#include "utils/serialize.hpp"

using namespace std;
using namespace utils::serialize;

serialize<string>::serialize(const string &obj):
    t(obj) {}

ostream &std::operator<<(ostream &os, const serialize<string> &s) {
    size_t sz = s.t.size();
    os.write(reinterpret_cast<const char *>(&sz), sizeof(sz));
    os.write(s.t.data(), sz);
    return os;
}

deserialize<string>::deserialize(string &obj):
    t(obj) {}

istream &std::operator>>(istream &is, deserialize<string> s) {
    size_t sz = 0;
    is.read(reinterpret_cast<char *>(&sz), sizeof(sz));

    char *buf = new char[sz + 1];
    is.read(buf, sz);
    buf[sz] = '\0';

    s.t = string(buf);

    delete buf;
    return is;
}

serialize<bool>::serialize(bool obj):
    t(obj) {}

ostream &std::operator<<(ostream &os, const serialize<bool> &s) {
    os.write(reinterpret_cast<const char *>(&s.t), sizeof(s.t));
    return os;
}

deserialize<bool>::deserialize(bool &obj):
    t(obj) {}

istream &std::operator>>(istream &is, deserialize<bool> s) {
    is.read(reinterpret_cast<char *>(&s.t), sizeof(s.t));
    return is;
}

serialize<int>::serialize(int obj):
    t(obj) {}

ostream &std::operator<<(ostream &os, const serialize<int> &s) {
    os.write(reinterpret_cast<const char *>(&s.t), sizeof(s.t));
    return os;
}

deserialize<int>::deserialize(int &obj):
    t(obj) {}

istream &std::operator>>(istream &is, deserialize<int> s) {
    is.read(reinterpret_cast<char *>(&s.t), sizeof(s.t));
    return is;
}
