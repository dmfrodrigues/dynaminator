#include "utils/io.hpp"

#include <filesystem>
#include <sstream>
#include <fstream>

using namespace std;

string utils::readWholeFile(const string &path) {
    ifstream ifs;
    ifs.exceptions(ifstream::badbit | ifstream::failbit);
    try {
        ifs.open(path);
    } catch (const ifstream::failure &e) {
        throw ifstream::failure("Could not open file " + path + " (full path:" + filesystem::canonical(path).string() + ")");
    }
    stringstream ss;
    ss << ifs.rdbuf();
    string all = ss.str();
    return all;
}
