#pragma once

#include <string>
#include <vector>

namespace utils {
bool ends_with(const std::string &s, const std::string &p);

std::vector<std::string> split(const std::string &s, const std::string &delim);
}  // namespace utils
