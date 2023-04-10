#include "utils/stringifier.hpp"

using namespace std;

int utils::stringifier<int>::fromString(const string &s){
    return atoi(s.c_str());
}

string utils::stringifier<int>::toString(const int &s){
    return to_string(s);
}

unsigned long utils::stringifier<unsigned long>::fromString(const string &s){
    return strtoul(s.c_str(), nullptr, 10);
}

string utils::stringifier<unsigned long>::toString(const unsigned long &s){
    return to_string(s);
}

float utils::stringifier<float>::fromString(const string &s){
    return (float)atof(s.c_str());
}

string utils::stringifier<float>::toString(const float &s){
    return to_string(s);
}

double utils::stringifier<double>::fromString(const string &s){
    return atof(s.c_str());
}

string utils::stringifier<double>::toString(const double &s){
    return to_string(s);
}

string utils::stringifier<string>::fromString(const string &s){
    return s;
}

string utils::stringifier<string>::toString(const string &s){
    return s;
}
