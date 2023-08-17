#include "utils/string.hpp"

using namespace std;

bool utils::ends_with(const string &s, const string &p) {
    return s.compare(s.length() - p.length(), p.length(), p) == 0;
}

vector<string> utils::split(const string &s, const string &delim) {
    vector<string> result;

    size_t start = 0;
    size_t end   = s.find(delim);

    while(end != string::npos) {
        result.emplace_back(s.substr(start, end - start));
        start = end + delim.length();
        end   = s.find(delim, start);
    }

    result.emplace_back(s.substr(start, end));

    return result;
}
