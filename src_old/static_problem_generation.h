// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file static_problem_generation.h
 *
 * @brief Contains functions for generating static planning problems; i.e. problems that do not change during the
 * execution of the planned path, and assume that the robot starts at a fixed start state, knowing the exact
 * configuration of the environment.
 */

#ifndef NEW_PLANNERS_STATIC_PROBLEM_GENERATION_H
#define NEW_PLANNERS_STATIC_PROBLEM_GENERATION_H

#include <moveit/robot_state/robot_state.h>
#include <json/value.h>

#include "AppleTreePlanningScene.h"

/**
 * A full definition of a planning problem, containing a start state and a planning scene,
 * including a set of goals and a set of obstacles.
 */
struct Problem {
	/// The start state of the robot.
	moveit::core::RobotState start;

	/// The planning scene, including a set of goals and a set of obstacles.
	AppleTreePlanningScene scene;
};

/**
 * @brief Generates a vector of planning problems, each containing a pair of JSON values and Problem objects.
 * Each planning problem features a single tree within a static, fully-known scene and is created using
 * a combination of model names and a random start state. The function generates multiple Problem instances
 * for each tree model by varying the randomly-generated start state.
 *
 * @param robot A shared pointer to a moveit::core::RobotModelConst object.
 * @param numRepetitions The number of repetitions to be performed for each problem, generating that many
 *        Problem instances for every tree model with a different randomly-generated start state each time.
 * @param modelNames A vector of model names used to create the planning problems.
 * @return A vector of pairs, each containing a JSON value and a Problem object representing a planning problem
 *         with a single tree in a static, fully-known scene.
 */
std::vector<std::pair<Json::Value, Problem>> generateStaticPlanningProblems(moveit::core::RobotModelConstPtr robot,
																			int numRepetitions,
																			const std::vector<std::string> &modelNames);

/**
 * @brief Generates a vector of planning problems, each containing a pair of JSON values and Problem objects.
 * The planning problems feature multiple trees within a static, fully-known scene.
 * The function creates a new problem instance for each repetition.
 *
 * For each problem, a subset of scenes is randomly picked from the available scenes,
 * each scene containing a single tree. These scenes are then merged into one combined scene.
 * The merging process arranges the trees from the individual scenes in a linear pattern,
 * adding an offset to the x-coordinate of the positions of all the collision objects in each scene.
 * This results in a scene with multiple trees arranged in a row, with a constant separation distance.
 *
 * A random start state is generated for each problem.
 *
 * @param robot A shared pointer to a moveit::core::RobotModel object.
 * @param num_reps The number of repetitions to be performed for each problem, generating that many
 *        Problem instances with different randomly-generated start states and combined scenes each time.
 * @param model_names A vector of model names used to create the planning scenes.
 * @param n_per_scene The number of scenes to be picked and merged for each problem.
 * @return A vector of pairs, each containing a JSON value and a Problem object representing a planning problem
 *         with multiple trees in a static, fully-known scene.
 */
std::vector<std::pair<Json::Value, Problem>> generateStaticOrchardPlanningProblems(const moveit::core::RobotModelPtr &robot,
									  int num_reps,
									  const std::vector<std::string> &model_names,
									  int n_per_scene);

#endif //NEW_PLANNERS_STATIC_PROBLEM_GENERATION_H
