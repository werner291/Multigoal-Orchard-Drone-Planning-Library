// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <ostream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "benchmark_function_macros.h"


#include <iostream>
#include <fstream>
#include <map>
#include <json/value.h>
#include <json/writer.h>

std::map<std::string, BenchmarkFn> benchmarks;

//
// Created by werner on 14-2-24.
//
int main(int argc, char **argv) {
	std::cout << "Multigoal Orchard Drone Planning Library -- Benchmarks" << std::endl;
	std::cout << "Version: " << GIT_HASH << std::endl;

#ifdef NDEBUG
	std::cout << "Running in release mode" << std::endl;
#else
	std::cout << "Running in debug mode. WARNING: This may be slower than fair for a benchmark." << std::endl;
#endif

	// Blank line
	std::cout << std::endl;

	// Check the command line args for a visualization name:
	if (argc >= 2) {
		std::string visualization_name = argv[1];
		if (benchmarks.find(visualization_name) == benchmarks.end()) {
			std::cout << "Unknown visualization: " << visualization_name << std::endl;
			return 1;
		}


		std::cout << "Running benchmark " << visualization_name << std::endl;

		Json::Value root;

		// Record the starting time:
		auto start = std::chrono::high_resolution_clock::now();

		benchmarks[visualization_name](root["results"]);

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
		root["benchmark"] = visualization_name;
		root["duration_ms"] = total_elapsed;
		root["commit"] = GIT_HASH;
#ifdef NDEBUG
		root["debug"] = false;
#else
		root["debug"] = true;
#endif

		// Format the filename with the ISO 8601 timestamp, benchmark name and DEBUG flag:
		std::stringstream filename;
		filename << "benchmark_" << visualization_name << "_" << current_time_iso;

#ifndef NDEBUG
		filename << "_DEBUG";
#endif
		filename << ".json";

		// Dump the JSON to the file:
		std::ofstream file(filename.str());
		file << root;
		file.close();

		std::cout << "Results written to " << filename.str() << std::endl;
	} else {
		// Print available benchmarks
		std::cout << "No benchmark chosen. Available benchmarks:" << std::endl;
		for (const auto &[name, _]: benchmarks) {
			std::cout << "- " << name << std::endl;
		}
		std::cout << "Please re-run with the name of a benchmark as an argument." << std::endl;
	}
}
