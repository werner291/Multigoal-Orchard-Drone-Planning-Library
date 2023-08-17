//
// Created by werner on 6-3-23.
//

#include "InitialOrbitPlanner.h"
#include "../shell_space/SphereShell.h"
#include "../shell_space/OmplShellSpace.h"
#include "../utilities/ompl_tools.h"
#include "../utilities/enclosing_sphere.h"

std::optional<DynamicMultiGoalPlanner::PathSegment>
InitialOrbitPlanner::plan_initial(const ompl::base::SpaceInformationPtr &si,
								  const ompl::base::State *start,
								  const std::vector<ompl::base::GoalPtr> &goals,
								  const AppleTreePlanningScene &planning_scene,
								  double padding) {

	batch = goals;

	// Get a shell space.
	{
		auto statespace = std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace());
		assert(statespace != nullptr);

		auto sphere = utilities::compute_enclosing_sphere_around_leaves(*planning_scene.scene_msg, padding);

		moveit::core::RobotState robot_state(statespace->getRobotModel());
		statespace->copyToRobotState(robot_state, start);

		Eigen::Vector3d ee_pos = robot_state.getGlobalLinkTransform("end_effector").translation();

		double init_theta = std::atan2(ee_pos.y() - sphere.center.y(), ee_pos.x() - sphere.center.x());

		const size_t SEGMENTS = 32;

		RobotPath path;
		path.waypoints.push_back(robot_state);

		for (size_t i = 0; i < SEGMENTS; i++) {

			double theta = init_theta + 2 * M_PI * (double) i / (double) SEGMENTS;

			Eigen::Vector3d pos(
					sphere.center.x() + sphere.radius * std::cos(theta),
					sphere.center.y() + sphere.radius * std::sin(theta),
					sphere.center.z()
					);

			Eigen::Vector3d armvec(-std::cos(theta), -std::sin(theta), 0);

			path.waypoints.push_back(robotStateFromPointAndArmvec(statespace->getRobotModel(), pos, armvec));

		}

		this->initial_orbit_path = robotPathToOmplPath(path, si);

		assert(this->initial_orbit_path->check());

		return initial_orbit_path;

	}

}

std::optional<DynamicMultiGoalPlanner::PathSegment>
InitialOrbitPlanner::replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
										   const ompl::base::State *current_state,
										   const AppleTreePlanningScene &planning_scene) {

//	std::cout << "Replanning after initial orbit." << std::endl;

	if (initial_orbit_path.has_value()) {

		initial_orbit_path.reset();

		auto path = after_planner->plan_initial(si, current_state, batch, planning_scene, 0.5);

		assert(!path || si->distance(path->getState(0), current_state) < 0.1);

		return path;

	} else {

		return after_planner->replan_after_path_end(si, current_state, planning_scene);

	}

}

std::optional<DynamicMultiGoalPlanner::PathSegment>
InitialOrbitPlanner::replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
											const ompl::base::State *current_state,
											const ompl::base::GoalPtr &new_goal,
											const PathInterrupt &interrupt,
											const AppleTreePlanningScene &planning_scene) {

//	std::cout << "InitialOrbitPlanner::replan_after_discovery" << std::endl;

	if (initial_orbit_path.has_value()) {

		utilities::truncatePathToInterrupt(*initial_orbit_path, interrupt);

		batch.push_back(new_goal);

		assert(si->distance(initial_orbit_path->getState(0), current_state) < 0.1);

		return initial_orbit_path;
	} else {
		return after_planner->replan_after_discovery(si, current_state, new_goal, interrupt, planning_scene);
	}

}

std::optional<DynamicMultiGoalPlanner::PathSegment>
InitialOrbitPlanner::replan_after_removal(const ompl::base::SpaceInformationPtr &si,
										  const ompl::base::State *current_state,
										  const ompl::base::GoalPtr &removed_goal,
										  const PathInterrupt &interrupt,
										  const AppleTreePlanningScene &planning_scene) {

	if (initial_orbit_path.has_value()) {
		utilities::truncatePathToInterrupt(*initial_orbit_path, interrupt);

		auto batch_itr = std::find(batch.begin(), batch.end(), removed_goal);
		assert(batch_itr != batch.end());
		batch.erase(batch_itr);

		return initial_orbit_path;
	} else {
		return after_planner->replan_after_removal(si, current_state, removed_goal, interrupt, planning_scene);
	}

}

InitialOrbitPlanner::InitialOrbitPlanner(const std::shared_ptr<DynamicMultiGoalPlanner> &staticPlanner) : after_planner(staticPlanner) {
}
