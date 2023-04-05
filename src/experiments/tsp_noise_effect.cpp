// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <random>
#include <fstream>
#include <Eigen/Core>
#include <range/v3/all.hpp>
#include "../utilities/traveling_salesman.h"

#include <fstream>
#include <json/json.h>

// Define an enum for different noise types.
enum class NoiseType {
	ADDITIVE, MULTIPLICATIVE, POSITIVE, POSITIVE_MULTIPLICATIVE, NEGATIVE_MULTIPLICATIVE
};

// Convert the noise type enum to a string for use as a key in the JSON object.
std::string noise_type_to_string(NoiseType noise_type) {
	switch (noise_type) {
		case NoiseType::ADDITIVE:
			return "additive";
		case NoiseType::MULTIPLICATIVE:
			return "multiplicative";
		case NoiseType::POSITIVE:
			return "positive";
		case NoiseType::POSITIVE_MULTIPLICATIVE:
			return "positive_multiplicative";
		case NoiseType::NEGATIVE_MULTIPLICATIVE:
			return "negative_multiplicative";
		default:
			return "";
	}
}

// Calculate the distance between two points on a sphere with a specified noise type.
double calculate_distance(const Eigen::Vector3d &point1,
						  const Eigen::Vector3d &point2,
						  double noise_scale,
						  NoiseType noise_type,
						  std::normal_distribution<> &d,
						  std::mt19937 &gen) {
	double angle = std::acos(std::clamp(point1.dot(point2), -1.0, 1.0));
	double noise = noise_scale * d(gen);

	switch (noise_type) {
		case NoiseType::ADDITIVE:
			return angle + noise;
		case NoiseType::MULTIPLICATIVE:
			return angle * (1.0 + noise);
		case NoiseType::POSITIVE:
			return angle + std::abs(noise);
		case NoiseType::POSITIVE_MULTIPLICATIVE:
			return angle * (1.0 + std::abs(noise));
		case NoiseType::NEGATIVE_MULTIPLICATIVE:
			return std::max(angle * (1.0 - std::abs(noise)), 0.0);
		default:
			return angle;
	}
}

int main() {

	// Generate a set of random points on a sphere.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<> d(0, 1);

	Json::Value results;

	// Save noise scales in the results dictionary.
	Json::Value noise_scales(Json::arrayValue);
	for (double noise_scale: {0.0, 0.1, 0.2, 0.5, 1.0}) {
		noise_scales.append(noise_scale);
	}
	results["noise_scales"] = noise_scales;

	// Perform iterations.
	Json::Value iterations(Json::arrayValue);

	for (size_t iteration = 0; iteration < 200; iteration++) {

		if (iteration % 10 == 0) {
			std::cout << "Iteration " << iteration << std::endl;
		}

		// Generate random points on the sphere.
		std::vector<Eigen::Vector3d> points = ranges::views::iota(0, 100) | ranges::views::transform([&](int i) {
			return Eigen::Vector3d(d(gen), d(gen), d(gen)).normalized();
		}) | ranges::to<std::vector>();

		Json::Value iterationResults;

		// Iterate through different noise types.
		for (NoiseType noise_type: {NoiseType::ADDITIVE, NoiseType::MULTIPLICATIVE, NoiseType::POSITIVE,
									NoiseType::POSITIVE_MULTIPLICATIVE, NoiseType::NEGATIVE_MULTIPLICATIVE}) {
			// Iterate through different noise scales.
			for (double noise_scale: {0.0, 0.1, 0.2, 0.5, 1.0}) {

				auto order = tsp_open_end([](size_t i) { return 0.0; }, [&](size_t i, size_t j) {
					return calculate_distance(points[i], points[j], noise_scale, noise_type, d, gen);
				}, points.size());

				double length = 0.0;
				// Calculate the total length of the TSP route.
				for (size_t i = 0; i + 1 < order.size(); i++) {
					length += std::acos(std::clamp(points[order[i]].dot(points[order[i + 1]]), -1.0, 1.0));
				}

				iterationResults[noise_type_to_string(noise_type)].append(length);
			}
		}

		// Store the results for the current iteration.
		iterations.append(iterationResults);
	}

	// Save the iterations in the results dictionary.
	results["iterations"] = iterations;

	// Write the results to a JSON file.
	std::ofstream json_out("analysis/data/tsp_noise_effect.json");
	json_out << results;
	json_out.close();

	return 0;
}
