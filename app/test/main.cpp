#include <spdlog/sinks/sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <catch2/catch_session.hpp>
#include <iostream>

std::string baseDir      = "";
std::string benchmarkDir = "";

int main(int argc, char* argv[]) {
    // Logger
    std::shared_ptr<spdlog::sinks::sink> mySink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    mySink->set_level(spdlog::level::info);
    spdlog::default_logger()->sinks() = {mySink};

    // Setup
    using namespace Catch::Clara;

    assert(std::numeric_limits<double>::has_infinity);
    assert(std::numeric_limits<double>::is_iec559);

    Catch::Session session;

    // Based on https://github.com/catchorg/Catch2/blob/devel/docs/own-main.md#adding-your-own-command-line-options
    Parser cli = session.cli()
               | Opt(baseDir, "baseDir")["-d"]["--baseDir"]("Base data directory")
               | Opt(benchmarkDir, "benchmarkDir")["-b"]["--benchmarkDir"]("Benchmarks data directory");

    session.cli(cli);

    int result = session.run(argc, argv);

    // Clean-up...

    return result;
}
