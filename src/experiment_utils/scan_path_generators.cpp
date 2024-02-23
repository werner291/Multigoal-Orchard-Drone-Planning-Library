// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9-2-24.
//

#include "scan_path_generators.h"

#include "JsonMeta.h"

namespace mgodpl {
	std::vector<std::pair<std::string, std::vector<JsonMeta<ParametricPath>>>> gen_orbits(
			const math::Vec3d &fruit_center, const double target_radius) {
		std::vector<std::pair<std::string, std::vector<JsonMeta<mgodpl::ParametricPath>>>> orbit_types;

		{
			std::vector<mgodpl::JsonMeta<ParametricPath>> orbits;
			for (int i = 0; i <= 5; ++i) {
				double radius = target_radius + i * 0.2;
				Json::Value meta;
				meta["radius"] = radius;

				orbits.push_back(JsonMeta<ParametricPath>{
						.meta = meta,
						.data = fixed_radius_equatorial_orbit(fruit_center, radius)
				});
			}
			orbit_types.emplace_back("concentric_equatorial_orbits", orbits);
		}

		{
			std::vector<JsonMeta<ParametricPath>> orbits;
			for (int i = 0; i <= 5; ++i) {
				const double ascending_node_longitude = 0.0;
				double radius = target_radius + i * 0.2;
				Json::Value meta;
				meta["radius"] = radius;

				orbits.push_back(JsonMeta<ParametricPath>{
						.meta = meta,
						.data = polar_orbit(fruit_center, radius, ascending_node_longitude)
				});
			}
			orbit_types.emplace_back("concentric_polar_orbits", orbits);
		}

		{
			std::vector<JsonMeta<ParametricPath>> orbits;
			for (int j = 0; j <= 3; ++j) {
				double radius = target_radius + j * 0.1;
				for (int i = 1; i <= 3; ++i) {
					int turns = i;
					double height = i * 0.1;
					Json::Value meta;
					meta["radius"] = radius;
					meta["turns"] = turns;
					meta["height"] = height;

					orbits.push_back(JsonMeta<ParametricPath>{
							.meta = meta,
							.data = helical_path(fruit_center, radius, turns, height)
					});
				}
			}
			orbit_types.emplace_back("helical_orbits", orbits);
		}

		{
			std::vector<JsonMeta<ParametricPath>> orbits;
			for (int i = 0; i <= 2; ++i) {
				double radius = target_radius + i * 0.5;
				for (int j = 1; j <= 3; ++j) {
					double amplitude = j * 0.1;
					for (int k = 2; k <= 4; ++k) {
						int cycles = k;
						Json::Value meta;
						meta["radius"] = radius;
						meta["amplitude"] = amplitude;
						meta["cycles"] = cycles;

						orbits.push_back(JsonMeta<ParametricPath>{
								.meta = meta,
								.data = vertical_oscillation_path(fruit_center, radius, amplitude, cycles)
						});
					}
				}
			}
			orbit_types.emplace_back("vertical_oscillation_paths", orbits);
		}

		{
			std::vector<JsonMeta<ParametricPath>> orbits;
			for (int i = 0; i <= 2; ++i) {
				double radius = target_radius + i * 0.2;
				for (int j = 1; j <= 5; ++j) {
					double amplitude = j * 0.2;
					for (int k = 1; k <= 5; ++k) {
						int cycles = k;
						Json::Value meta;
						meta["radius"] = radius;
						meta["amplitude"] = amplitude;
						meta["cycles"] = cycles;

						orbits.push_back(JsonMeta<ParametricPath>{
								.meta = meta,
								.data = latitude_oscillation_path(fruit_center, radius, amplitude, cycles)
						});
					}
				}
			}
			orbit_types.emplace_back("latitude_oscillation_paths", orbits);
		}

		return orbit_types;
	}

	std::vector<JsonMeta<ParametricPath>> getOrbits(const math::Vec3d &fruit_center, double EYE_ORBIT_RADIUS) {
		// Generate a list of orbits.
		const auto orbit_types = mgodpl::gen_orbits(fruit_center, EYE_ORBIT_RADIUS);

		// Prompt the user to select the type of orbit.
		std::cout << "Choose orbit type: " << std::endl;
		for (int i = 0; i < orbit_types.size(); i++) {
			std::cout << i << ": " << orbit_types[i].first << std::endl;
		}

		std::cout << (orbit_types.size()) << ": All" << std::endl;

		int orbitType;

		// Loop until a valid orbit type is selected.
		while (true) {
			std::cout << "Choose orbit type: " << std::endl;
			for (int i = 0; i < orbit_types.size(); i++) {
				std::cout << i << ": " << orbit_types[i].first << std::endl;
			}
			std::cout << (orbit_types.size()) << ": All" << std::endl;

			std::cin >> orbitType;

			// If the user enters a valid orbit type, break the loop.
			if (orbitType >= 0 && orbitType <= orbit_types.size()) {
				break;
			}

			// If the user enters an invalid orbit type, prompt again.
			std::cout << "Invalid orbit type. Please try again." << std::endl;
		}

		std::vector<JsonMeta<ParametricPath>> orbits;

		// If the user selects all orbits, add all orbits to the return vector.
		if (orbitType == orbit_types.size()) {
			std::cout << "All orbits" << std::endl;
			for (const auto &orbit_type: orbit_types) {
				auto orbit = orbit_type.second;
				orbits.insert(orbits.end(), orbit.begin(), orbit.end());
			}
		}
			// If the user selects a specific orbit type, add only the orbits of that type to the return vector.
		else {
			std::cout << "Orbit type: " << orbit_types[orbitType].first << std::endl;
			orbits = orbit_types[orbitType].second;
		}

		// Return the selected orbits.
		return orbits;
	}

	ParametricPath getSingleOrbit(const math::Vec3d &fruit_center, const double base_radius) {

		// Generate a list of orbits.
		const std::vector<std::string> orbit_types = {
				"Equatorial circular",
				"Polar circular",
				"Helical",
				"Vertical oscillation",
				"Latitude oscillation"
		};

		// Prompt the user to select the type of orbit.
		std::cout << "Choose orbit type: " << std::endl;
		for (size_t i = 0; i < orbit_types.size(); i++) {
			std::cout << i << ": " << orbit_types[i] << std::endl;
		}

		size_t orbitType;

		// Loop until a valid orbit type is selected.
		while (true) {
			std::cin >> orbitType;

			// If the user enters a valid orbit type, break the loop.
			if (orbitType >= 0 && orbitType < orbit_types.size()) {
				break;
			}

			// If the user enters an invalid orbit type, prompt again.
			std::cout << "Invalid orbit type. Please try again." << std::endl;
		}

		// Generate the orbit with the provided parameters.
		switch (orbitType) {
			case 0: {
				// Prompt for the radius:
				double radius;
				std::cout << "Enter the radius of the orbit: ";
				std::cin >> radius;

				return fixed_radius_equatorial_orbit(fruit_center, radius * base_radius);
			}
				break;

			case 1: {
				// Prompt for the radius:
				double radius;
				std::cout << "Enter the radius of the orbit: ";
				std::cin >> radius;

				// Prompt for the longitude of the ascending node:
				double ascending_node_longitude;
				std::cout << "Enter the longitude of the ascending node: ";
				std::cin >> ascending_node_longitude;

				return polar_orbit(fruit_center, radius * base_radius, ascending_node_longitude);
			}
				break;

			case 2: {
				// Prompt for the radius, turns and height:
				double radius, height;
				int turns;
				std::cout << "Enter the radius of the orbit: ";
				std::cin >> radius;
				std::cout << "Enter the number of turns: ";
				std::cin >> turns;
				std::cout << "Enter the height: ";
				std::cin >> height;

				return helical_path(fruit_center, radius * base_radius, turns, height * base_radius);
			}
				break;

			case 3: {
				// Prompt for the radius, amplitude and cycles:
				double radius, amplitude;
				int cycles;
				std::cout << "Enter the radius of the orbit: ";
				std::cin >> radius;
				std::cout << "Enter the amplitude: ";
				std::cin >> amplitude;
				std::cout << "Enter the number of cycles: ";
				std::cin >> cycles;

				return vertical_oscillation_path(fruit_center, radius * base_radius, amplitude * base_radius, cycles);
			}
				break;

			case 4: {
				// Prompt for the radius, amplitude and cycles:
				double radius, amplitude;
				int cycles;
				std::cout << "Enter the radius of the orbit: ";
				std::cin >> radius;
				std::cout << "Enter the amplitude: ";
				std::cin >> amplitude;
				std::cout << "Enter the number of cycles: ";
				std::cin >> cycles;

				return latitude_oscillation_path(fruit_center, radius * base_radius, amplitude * base_radius, cycles);
			}
				break;

			default:
				throw std::runtime_error("Invalid orbit type");
		}

		throw std::runtime_error("Unreachable code");
	}


}
