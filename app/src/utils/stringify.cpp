#include "utils/stringify.hpp"

using namespace std;
using namespace utils::stringify;

int stringify<int>::fromString(const string &s){
    return atoi(s.c_str());
}

string stringify<int>::toString(const int &s){
    return to_string(s);
}

long stringify<long>::fromString(const string &s){
    return strtol(s.c_str(), nullptr, 10);
}

string stringify<long>::toString(const long &s){
    return to_string(s);
}

unsigned long stringify<unsigned long>::fromString(const string &s){
    return strtoul(s.c_str(), nullptr, 10);
}

string stringify<unsigned long>::toString(const unsigned long &s){
    return to_string(s);
}

float stringify<float>::fromString(const string &s){
    return (float)atof(s.c_str());
}

string stringify<float>::toString(const float &s){
    return to_string(s);
}

double stringify<double>::fromString(const string &s){
    return atof(s.c_str());
}

string stringify<double>::toString(const double &s){
    return to_string(s);
}

string stringify<string>::fromString(const string &s){
    return s;
}

string stringify<string>::toString(const string &s){
    return s;
}
