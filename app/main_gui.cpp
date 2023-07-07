#include <filesystem>

#include "UI/Simulator.hpp"

int main(int argc, char** argv) {
    assert(argc >= 2);

    std::filesystem::path configFilePath = argv[1];

    UI::Simulator simulator(configFilePath);

    simulator.run();

    return 0;
}
