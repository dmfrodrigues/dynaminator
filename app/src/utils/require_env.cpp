#include "utils/require_env.hpp"

#include <stdexcept>

using namespace std;
using namespace utils;

string utils::require_env(const string &name) {
    const char *value = getenv(name.c_str());
    if(value == nullptr) {
        throw runtime_error("Environment variable " + name + " is not set.");
    }
    return value;
}
