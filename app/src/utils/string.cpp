#include "utils/string.hpp"

bool utils::ends_with(const std::string &s, const std::string &p) {
    return s.compare(s.length() - p.length(), p.length(), p) == 0;
}
