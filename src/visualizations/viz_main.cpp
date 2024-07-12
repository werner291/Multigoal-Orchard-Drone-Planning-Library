// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <iostream>
#include <map>
#include <vector>

#include "../visualization/visualization_function_macros.h"

std::map<std::string, VisFn> visualizations;

//
// Created by werner on 14-2-24.
//
int main(int argc, char **argv) {
	// Check the command line args for a visualization name:
	if (argc > 1) {
		std::string visualization_name = argv[1];
		if (visualizations.find(visualization_name) == visualizations.end()) {
			std::cerr << "Unknown visualization: " << visualization_name << std::endl;
			return 1;
		}

		mgodpl::SimpleVtkViewer viewer;

		// Check if argv[2] is "record"
		if (argc > 2 && std::string(argv[2]) == "record") {
			std::cout << "Recording to " << visualization_name << ".ogv" << std::endl;
			viewer.startRecording(visualization_name + ".ogv");
		}

		visualizations[visualization_name](viewer);
		return 0;
	} else {
		// Print a list of visualizations with a number:
		std::cout << "Available visualizations:" << std::endl;
		int i = 0;
		std::vector<std::string> visualization_names;
		for (const auto &[name, _]: visualizations) {
			std::cout << i << ": " << name << std::endl;
			visualization_names.push_back(name);
			i++;
		}

		// Print a prompt
		std::cout << "Enter a number, and add \"record\" to record the visualization to a file." << std::endl;

		// Wait for a numbered input
		int choice;
		std::cin >> choice;

		// Make sure that the choice is valid
		if (choice < 0 || choice >= visualization_names.size()) {
			std::cerr << "Invalid choice" << std::endl;
			return 1;
		}

		mgodpl::SimpleVtkViewer viewer;

		// Check if the user wants to record the visualization
		std::string record;
		std::getline(std::cin, record);
		// Trim leading and trailing spaces
		record.erase(0, record.find_first_not_of(' ')); // leading spaces
		record.erase(record.find_last_not_of(' ') + 1); // trailing spaces
		if (record == "record") {
			std::stringstream filename;
			filename << visualization_names[choice] << ".ogv";

			// Get the current working directory


			std::cout << "Recording to ";

			// Full working dir:
			std::cout << std::filesystem::current_path().generic_string() << "/" << filename.str() << std::endl;

			viewer.startRecording(filename.str());
		} else if (!record.empty()) {
			std::cerr << "Invalid input: " << record << std::endl;
			std::exit(1);
		}

		// Run that visualization
		visualizations[visualization_names[choice]](viewer);
	}
}
