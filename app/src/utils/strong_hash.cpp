#include "utils/strong_hash.hpp"

using namespace std;

size_t utils::strong_hash<long>::operator()(long x) const {
    // From:
    // - https://stackoverflow.com/a/12996028/12283316
    x = (x ^ (x >> 30)) * (long)(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) * (long)(0x94d049bb133111eb);
    x = x ^ (x >> 31);
    return x;
}
