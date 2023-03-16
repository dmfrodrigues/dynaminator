#include "utils/io.hpp"

#include <sstream>
#include <fstream>

using namespace std;

string utils::readWholeFile(const string &path) {
    ifstream ifs;
    ifs.exceptions(ifstream::badbit | ifstream::failbit);
    ifs.open(path);
    stringstream ss;
    ss << ifs.rdbuf();
    string all = ss.str();
    return all;
}
