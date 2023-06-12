// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "run_static.h"
#include "utilities/mesh_utils.h"

Json::Value
runPlannerOnStaticProblem(const StaticPlannerAllocatorFn &planner, const Problem &problem) {

	double translation_bound = 0.0;

	// Get the AABB of the apple tree.
	for (const auto& obj : problem.scene.scene_msg->world.collision_objects) {

		assert(obj.meshes.size() == 1);
		assert(obj.mesh_poses.size() == 1);

		auto aabb = mesh_aabb(obj.meshes[0]);

		translation_bound = std::max(translation_bound, aabb.max().x());
		translation_bound = std::max(translation_bound, aabb.max().y());
		translation_bound = std::max(translation_bound, aabb.max().z());
		translation_bound = std::max(translation_bound, std::abs(aabb.min().x()));
		translation_bound = std::max(translation_bound, std::abs(aabb.min().y()));
	}

	translation_bound += 2.0;

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(problem.start.getRobotModel(), translation_bound);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, problem.scene);

	// Allocate the planner.
	auto ompl_planner = planner(si);

	ompl::base::ScopedState<> start_state(ss);
	ss->copyToOMPLState(start_state.get(), problem.start);

	auto goals = problem.scene.apples |
				 ranges::views::transform([&](const auto &apple) -> ompl::base::GoalPtr {
					 return std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
				 }) | ranges::to_vector;

	// use OMPL non-terminating condition
	auto ptc = ompl::base::timedPlannerTerminationCondition(200);

	// Record the start time.
	auto start_time = std::chrono::high_resolution_clock::now();
	auto eval = ompl_planner->plan(si, start_state.get(), goals, problem.scene, ptc);
	auto end_time = std::chrono::high_resolution_clock::now();

	// Check whether the path segments actually connect to each other.
	for (int i = 0; i + 1 < eval.segments.size(); ++i) {
		const auto &segment = eval.segments[i];
		const auto &next_segment = eval.segments[i + 1];
		assert(segment.path_.getStateCount() > 0);
		assert(si->distance(segment.path_.getState(segment.path_.getStateCount() - 1),
							next_segment.path_.getState(0)) < 1e-3);
	}

	Json::Value result;
	result["n_visited"] = eval.segments.size();
	result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	result["total_path_length"] = eval.length();

	return result;

}
