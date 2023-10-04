// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vector>
#include <eigen3/Eigen/Core>
#include <random>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <json/json.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include "../ORToolsTSPMethods.h"

/**
* @brief Initializes two TSP methods for ORTools, paired with names for later reference.
*/
std::array<std::pair<std::string, ORToolsTSPMethods>, 2> initTSPMethods() {
	ORToolsTSPMethods lci(ORToolsTSPMethods::LEAST_COSTLY_INSERT); // Create a TSP method using the least costly insert heuristic.
	ORToolsTSPMethods fre(ORToolsTSPMethods::FULL_REORDER); // Create a TSP method using the full reorder heuristic.

	std::array<std::pair<std::string, ORToolsTSPMethods>, 2> methods = { // Create an std::array of pairs of strings and ORToolsTSPMethods, and initialize it with the two TSP methods.
			std::pair{"LCI", lci}, std::pair{"FRE", fre}};

	return methods; // Return the std::array of TSP methods.
}

/**
* @brief Computes the cost of a TSP solution.
*
* This function computes the cost of a TSP solution, given the starting point and a vector of points.
*
* @param start The starting point of the TSP solution.
* @param points The vector of points that make up the TSP solution.
* @return The cost of the TSP solution.
*/
double solution_cost(const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &points) {
	double init_cost = (start -
						points[0]).norm(); // Compute the initial cost as the distance between the starting point and the first point in the vector.

	for (size_t i = 0; i < points.size() -
						   1; i++) { // Loop over the remaining points in the vector, computing the cost of each segment.
		init_cost += (points[i] - points[i + 1]).norm();
	}

	return init_cost; // Return the total cost of the TSP solution.
}

/**
* @brief Computes the angle between two unit vectors.
*
* This function computes the angle between two unit vectors, using the dot product.
*
* @param a The first unit vector.
* @param b The second unit vector.
* @return The angle between the two unit vectors, in radians.
*/
double angle_between_unit(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
	return std::acos(a.dot(b)); // Compute the angle between the two unit vectors using the dot product, and return it in radians.
}

/**
* @brief Reorders the points of a TSP using a specified method and starting point.
*
* This function reorders the points of a TSP using the specified method and starting point, by computing an initial ordering based on the angles between the points and the starting point, and then applying incremental insertions to improve the ordering.
*
* @param method The TSP method to use for reordering.
* @param start The starting point of the TSP.
* @param points The vector of points to reorder.
*/
void
reorder_initial(const ORToolsTSPMethods &method, const Eigen::Vector3d &start, std::vector<Eigen::Vector3d> &points) {
	std::vector<size_t> ordering = method.initial_ordering(100, [&](size_t i, size_t j) {
		return angle_between_unit(points[i], points[j]);
	}, [&](size_t i) {
		return angle_between_unit(start, points[i]);
	});

	points = ordering | ranges::views::transform([&](size_t i) { // Reorder the points using incremental insertions.
		return points[i];
	}) | ranges::to_vector;
}

/**
* @brief Applies an incremental TSP reordering to a set of points.
*
* This function applies an incremental TSP reordering to a set of points, using a set of insertion points and a new point to insert.
*
* @param points The vector of points to reorder (will be reordered in place).
* @param new_point The new point to insert.
* @param update The set of insertion points.
*/
void apply_reorder_with_insert(std::vector<Eigen::Vector3d> &points,
							   const Eigen::Vector3d &new_point,
							   std::vector<IncrementalTSPMethods::NewOrderingEntry> &update) {
	points = update |
			 ranges::views::transform([&](const IncrementalTSPMethods::NewOrderingEntry &e) { // Apply the incremental insertions.
				 if (std::holds_alternative<IncrementalTSPMethods::FromOriginal>(e)) {
					 return points[std::get<IncrementalTSPMethods::FromOriginal>(e).index];
				 } else {
					 return new_point;
				 }
			 }) | ranges::to_vector;
}


/**
 * @brief Applies a TSP reordering to a vector of points, given a new point to insert.
 *
 * @param points 		The vector of points to apply the TSP reordering to. (will be reordered in place)
 * @param new_point 	The new point to insert.
 * @param update 		The update that needs to be applied to the TSP solver.
 */
void update_order(const ORToolsTSPMethods &method,
				  const Eigen::Vector3d &start,
				  std::vector<Eigen::Vector3d> &points,
				  Eigen::Vector3d &new_point) {

	auto update = method.update_ordering_with_insertion(points.size(),
														[&](const IncrementalTSPMethods::NewOrderingEntry &a,
															const IncrementalTSPMethods::NewOrderingEntry &b) {

															// Look up the points according to whether they are from the original vector or the new point.
															auto pta = std::holds_alternative<IncrementalTSPMethods::FromOriginal>(
																	a)
																	   ? points[std::get<IncrementalTSPMethods::FromOriginal>(
																			a).index] : new_point;
															auto ptb = std::holds_alternative<IncrementalTSPMethods::FromOriginal>(
																	b)
																	   ? points[std::get<IncrementalTSPMethods::FromOriginal>(
																			b).index] : new_point;

															// Compute the angle between the two points and return as the cost.
															return angle_between_unit(pta, ptb);

														},
														[&](const IncrementalTSPMethods::NewOrderingEntry &a) {

															// Same here, but for only one point.
															// The other point is implicitly the new point.
															auto pta = std::holds_alternative<IncrementalTSPMethods::FromOriginal>(
																	a)
																	   ? points[std::get<IncrementalTSPMethods::FromOriginal>(
																			a).index] : new_point;

															// Compute the angle between the two points and return as the cost.
															return angle_between_unit(start, pta);

														});

	// Apply the update to the vector of points.
	apply_reorder_with_insert(points, new_point, update);
}

int main() {

	// Initialize a normal distribution and a random number generator
	std::normal_distribution<double> normal(0, 1);
	auto rng = std::mt19937{std::random_device{}()};

	// Initialize TSP methods
	auto methods = initTSPMethods();

	// Create a JSON object to store the results
	Json::Value results;

	// Repeat the experiment 50 times
	for (size_t rep = 0; rep < 50; rep++) {

		std::cout << "Rep: " << rep << std::endl;

		// For each TSP method
		for (const auto &[name, method]: methods) {

			// Generate a random start point
			Eigen::Vector3d start = Eigen::Vector3d(normal(rng), normal(rng), normal(rng)).normalized();

			// Generate 100 random points
			std::vector<Eigen::Vector3d> points = ranges::views::iota(0, 100) | ranges::views::transform([&](int i) {
				return Eigen::Vector3d(normal(rng), normal(rng), normal(rng)).normalized();
			}) | ranges::to_vector;

			// Create a JSON object to store the results of this TSP method
			Json::Value run_result;
			run_result["name"] = name;

			// Measure the time and cost of the initial ordering
			auto start_time = std::chrono::high_resolution_clock::now();
			reorder_initial(method, start, points);
			auto end_time = std::chrono::high_resolution_clock::now();

			run_result["initial_ordering_time"] = std::chrono::duration_cast<std::chrono::microseconds>(
					end_time - start_time).count();
			run_result["initial_ordering_cost"] = solution_cost(start, points);

			// Generate 100 random points and incrementally add them
			start_time = std::chrono::high_resolution_clock::now();

			for (size_t i = 0; i < 100; i++) {

				// Generate a new random point
				Eigen::Vector3d new_point = Eigen::Vector3d(normal(rng), normal(rng), normal(rng)).normalized();

				// Update the ordering with the new point
				auto update_start_time = std::chrono::high_resolution_clock::now();
				update_order(method, start, points, new_point);
				auto update_end_time = std::chrono::high_resolution_clock::now();

				// Measure the time and cost of the updated ordering
				Json::Value update_result;
				update_result["update_ordering_time"] = std::chrono::duration_cast<std::chrono::microseconds>(
						update_end_time - update_start_time).count();
				update_result["update_ordering_cost"] = solution_cost(start, points);

				// Append the results of this update to the JSON object
				run_result["updates"].append(update_result);
			}

			// Append the results of this TSP method to the overall JSON object
			results.append(run_result);
		}
	}

	// Write the results to a JSON file
	std::ofstream out("analysis/tsp_test_results.json");
	out << results;
	out.close();
}
