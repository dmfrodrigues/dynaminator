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

double utils::stringifier<double>::fromString(const string &s){
    return atof(s.c_str());
}

string utils::stringifier<double>::toString(const double &s){
    return to_string(s);
}
