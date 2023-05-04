#include "utils/stringify.hpp"

#include <cassert>

using namespace std;
using namespace utils::stringify;

int stringify<int>::fromString(const string &s) {
    return atoi(s.c_str());
}

string stringify<int>::toString(const int &s) {
    return to_string(s);
}

long stringify<long>::fromString(const string &s) {
    return strtol(s.c_str(), nullptr, 10);
}

string stringify<long>::toString(const long &s) {
    return to_string(s);
}

unsigned long stringify<unsigned long>::fromString(const string &s) {
    return strtoul(s.c_str(), nullptr, 10);
}

string stringify<unsigned long>::toString(const unsigned long &s) {
    return to_string(s);
}

bool stringify<bool>::fromString(const string &s) {
    if(s == "1")
        return true;
    else if(s == "0")
        return false;
    else
        throw runtime_error("Invalid bool string: " + s);
}

string stringify<bool>::toString(const bool &t) {
    return t ? "1" : "0";
}

float stringify<float>::fromString(const string &s) {
    return (float)atof(s.c_str());
}

string stringify<float>::toString(const float &s) {
    return to_string(s);
}

double stringify<double>::fromString(const string &s) {
    return atof(s.c_str());
}

string stringify<double>::toString(const double &s) {
    return to_string(s);
}

string stringify<string>::fromString(const string &s) {
    return s;
}

string stringify<string>::toString(const string &s) {
    return s;
}

string stringify<vector<bool>>::toString(const std::vector<bool> &t) {
    string s(t.size(), '-');

    for(size_t i = 0; i < t.size(); ++i) {
        string bs = stringify<bool>::toString(t[i]);
        assert(bs.size() == 1);
        s[i] = bs[0];
    }

    return s;
}

vector<bool> stringify<vector<bool>>::fromString(const std::string &s) {
    std::vector<bool> ret(s.size());

    for(size_t i = 0; i < s.size(); ++i) {
        ret[i] = stringify<bool>::fromString(s.substr(i, 1));
    }

    return ret;
}
