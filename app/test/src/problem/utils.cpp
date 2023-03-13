#include "test/utils.hpp"

using namespace std;

filesystem::path getExePath(){
    return filesystem::canonical("/proc/self/exe");
}
