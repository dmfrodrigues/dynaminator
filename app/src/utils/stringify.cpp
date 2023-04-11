#include "utils/stringify.hpp"

using namespace std;

int utils::stringify<int>::fromString(const string &s){
    return atoi(s.c_str());
}

string utils::stringify<int>::toString(const int &s){
    return to_string(s);
}

long utils::stringify<long>::fromString(const string &s){
    return strtol(s.c_str(), nullptr, 10);
}

string utils::stringify<long>::toString(const long &s){
    return to_string(s);
}

unsigned long utils::stringify<unsigned long>::fromString(const string &s){
    return strtoul(s.c_str(), nullptr, 10);
}

string utils::stringify<unsigned long>::toString(const unsigned long &s){
    return to_string(s);
}

float utils::stringify<float>::fromString(const string &s){
    return (float)atof(s.c_str());
}

string utils::stringify<float>::toString(const float &s){
    return to_string(s);
}

double utils::stringify<double>::fromString(const string &s){
    return atof(s.c_str());
}

string utils::stringify<double>::toString(const double &s){
    return to_string(s);
}

string utils::stringify<string>::fromString(const string &s){
    return s;
}

string utils::stringify<string>::toString(const string &s){
    return s;
}
