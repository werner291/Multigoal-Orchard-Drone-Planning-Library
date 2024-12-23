// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <boost/date_time/posix_time/posix_time.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iostream>
#include <json/value.h>
#include <json/writer.h>
#include <map>
#include <map>
#include <ostream>
#include <sstream>
#include <vector>

#include "../visualization/visualization_function_macros.h"
#include "benchmark_function_macros.h"

std::map<std::string, BenchmarkFn> benchmarks;
std::map<std::string, VisFn> visualizations;

int main(int argc, char **argv) {
    std::cout << "Multigoal Orchard Drone Planning Library" << std::endl;
    std::cout << "Version: " << GIT_HASH << std::endl;

#ifdef NDEBUG
    std::cout << "Running in release mode" << std::endl;
#else
    std::cout << "Running in debug mode. WARNING: This may be slower than fair for a benchmark." << std::endl;
#endif

    // Blank line
    std::cout << std::endl;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <benchmark|visualization> [benchmark_name|visualization_name]" <<
                std::endl;
        return 1;
    }

    std::string mode = argv[1]; // "benchmark" or "visualization"

    if (mode == "benchmark") {
        if (argc < 3) {
            std::cout << "No benchmark chosen. Available benchmarks:" << std::endl;
            for (const auto &[name, _]: benchmarks) {
                std::cout << "- " << name << std::endl;
            }
            std::cout << "Please re-run with the name of a benchmark as an argument." << std::endl;
            return 1;
        }

        std::string benchmark_name = argv[2];
        if (benchmarks.find(benchmark_name) == benchmarks.end()) {
            std::cerr << "Unknown benchmark: " << benchmark_name << std::endl;
            return 1;
        }

        std::cout << "Running benchmark " << benchmark_name << std::endl;

        Json::Value root;

        // Record the starting time:
        auto start = std::chrono::high_resolution_clock::now();

        benchmarks[benchmark_name](root["results"]);

        // Record the ending time:
        auto end = std::chrono::high_resolution_clock::now();

        // Calculate the duration:
        auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // Print the duration:
        std::cout << "Total time: " << total_elapsed << "ms" << std::endl;

        // Write the JSON to a file, formatted with an ISO 8601 timestamp, the name of the benchmark, and a DEBUG flag:
        std::string current_time_iso = boost::posix_time::to_iso_extended_string(
            boost::posix_time::second_clock::local_time());

        root["finished_at"] = current_time_iso;
        root["benchmark"] = benchmark_name;
        root["duration_ms"] = total_elapsed;
        root["commit"] = GIT_HASH;
#ifdef NDEBUG
        root["debug"] = false;
#else
        root["debug"] = true;
#endif

        // Format the filename with the ISO 8601 timestamp, benchmark name and DEBUG flag:
        std::stringstream filename;
        filename << "analysis/data/benchmark_" << benchmark_name << "_" << current_time_iso;

#ifndef NDEBUG
        filename << "_DEBUG";
#endif
        filename << ".json";

        // Dump the JSON to the file:
        std::ofstream file(filename.str());
        file << root;
        file.close();

        std::cout << "Results written to " << filename.str() << std::endl;
    } else if (mode == "visualization") {
        if (argc < 3) {
            std::cout << "Available visualizations:" << std::endl;
            int i = 0;
            std::vector<std::string> visualization_names;
            for (const auto &[name, _]: visualizations) {
                std::cout << i << ": " << name << std::endl;
                visualization_names.push_back(name);
                i++;
            }

            std::cout << "Enter a number or a name, and optionally add 'record' to record the visualization." <<
                    std::endl;

            // Wait for a numbered input or name
            std::string input;
            std::cin >> input;

            size_t choice;
            std::stringstream ss(input);
            if (!(ss >> choice)) {
                // If the input string cannot be parsed to an integer
                auto it = std::find_if(visualization_names.begin(),
                                       visualization_names.end(),
                                       [&input](const std::string &name) {
                                           return name.find(input) != std::string::npos;
                                       });
                if (it == visualization_names.end()) {
                    std::cerr << "Invalid choice" << std::endl;
                    return 1;
                }
                choice = std::distance(visualization_names.begin(), it);
            } else {
                if (choice < 0 || choice >= visualization_names.size()) {
                    std::cerr << "Invalid choice" << std::endl;
                    return 1;
                }
            }

            mgodpl::SimpleVtkViewer viewer;

            std::string record;
            std::getline(std::cin, record);
            record.erase(0, record.find_first_not_of(' '));
            record.erase(record.find_last_not_of(' ') + 1);

            if (record == "record") {
                std::stringstream filename;
                filename << visualization_names[choice] << ".ogv";
                std::cout << "Recording to " << filename.str() << std::endl;
                viewer.startRecording(filename.str());
            } else if (!record.empty()) {
                std::cerr << "Invalid input: " << record << std::endl;
                return 1;
            }

            visualizations[visualization_names[choice]](viewer);
        } else {
            std::string visualization_name = argv[2];
            if (visualizations.find(visualization_name) == visualizations.end()) {
                std::cerr << "Unknown visualization: " << visualization_name << std::endl;
                return 1;
            }

            mgodpl::SimpleVtkViewer viewer;

            if (argc > 3 && std::string(argv[3]) == "record") {
                std::cout << "Recording to " << visualization_name << ".ogv" << std::endl;
                viewer.startRecording(visualization_name + ".ogv");
            }

            visualizations[visualization_name](viewer);
        }
    } else {
        std::cerr << "Invalid mode. Please specify 'benchmark' or 'visualization' as the first argument." << std::endl;
        return 1;
    }

    return 0;
}
