#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/for_each.hpp>
#include <moveit/robot_state/conversions.h>
#include "ExperimentVisualTools.h"
#include "procedural_tree_generation.h"
#include "probe_retreat_move.h"
#include "utilities/msgs_utilities.h"

rclcpp::QoS default_qos() {
	rclcpp::QoS qos(1);
	qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	return qos;
}

ExperimentVisualTools::ExperimentVisualTools() : Node("new_planners") {

}

void ExperimentVisualTools::publishPlanningScene(const moveit_msgs::msg::PlanningScene &scene_msg) {

	auto pub = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", default_qos());

	pub->publish(scene_msg);

	publisher_handles.push_back(pub);

}

geometry_msgs::msg::Point pointEigenToMsg(const Eigen::Vector3d& p) {
	geometry_msgs::msg::Point msg;
	msg.x = p.x();
	msg.y = p.y();
	msg.z = p.z();
	return msg;
}

void ExperimentVisualTools::dumpProjections(const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
											const std::string &topic_name) {

	std_msgs::msg::ColorRGBA color;

	ompl::RNG rng;
	color.a = 1.0;
	color.r = (float) rng.uniform01();
	color.g = (float) rng.uniform01();
	color.b = (float) rng.uniform01();

	auto sphere_projections = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, default_qos());

	visualization_msgs::msg::MarkerArray projections_msg;

	visualization_msgs::msg::Marker balls;
	balls.header.frame_id = "world";
	balls.type = visualization_msgs::msg::Marker::POINTS;
	balls.points.resize(apples.size());
	balls.color = color;
	balls.scale.x = 0.05;
	balls.scale.y = 0.05;
	balls.scale.z = 0.05;
	balls.pose.orientation.w = 1.0;

	visualization_msgs::msg::Marker lines;
	lines.header.frame_id = "world";
	lines.id = 1;
	lines.type = visualization_msgs::msg::Marker::LINE_LIST;
	lines.scale.x = 0.01;
	lines.color = color;
	lines.pose.orientation.w = 1.0;
	lines.points.resize(2 * apples.size());

	for (const auto &[apple_id, approach_pair]: apples | ranges::views::enumerate) {

		const auto& approach_path = approach_pair.second;

		auto ss = approach_path.getSpaceInformation()
				->getStateSpace()
				->as<ompl_interface::ModelBasedStateSpace>();

		auto approach_state= approach_path.getState(0);
		moveit::core::RobotState approach_state_robot_state(ss->getRobotModel());
		ss->copyToRobotState(approach_state_robot_state, approach_state);
		Eigen::Vector3d ee_pos = approach_state_robot_state.getGlobalLinkTransform("end_effector").translation();

		balls.points[apple_id] = pointEigenToMsg(ee_pos);

		lines.points[apple_id * 2] = pointEigenToMsg(approach_pair.first.center);
		lines.points[apple_id * 2 + 1] = pointEigenToMsg(ee_pos);

	}

	projections_msg.markers.push_back(balls);
	projections_msg.markers.push_back(lines);

	sphere_projections->publish(projections_msg);

	publisher_handles.push_back(sphere_projections);
}

void ExperimentVisualTools::pincushion(const std::vector<std::pair<ompl::base::Goal *, ompl::base::State *>> &apples,
									   const ompl::base::StateSpace* space,
									   const std::string &topic_name) {

	auto pairs = apples | ranges::views::transform([&](const auto &pair) -> std::pair<Apple, moveit::core::RobotState> {

		auto ss = space->as<ompl_interface::ModelBasedStateSpace>();
		moveit::core::RobotState rs(ss->getRobotModel());
		ss->copyToRobotState(rs, pair.second);

		ompl::base::Goal * goal = pair.first;
		Eigen::Vector3d apple_pos = goal->as<DroneEndEffectorNearTarget>()->getTarget();

		Apple apple {
				.center= apple_pos
		};

		return std::make_pair(
				apple,
				rs
		);

	}) | ranges::to_vector;

	pincushion(
			pairs,
			topic_name
			);

}

void ExperimentVisualTools::pincushion(const std::vector<std::pair<Apple, moveit::core::RobotState>> &apples,
									   const std::string &topic_name) {

	std_msgs::msg::ColorRGBA color;

	ompl::RNG rng;
	color.a = 1.0;
	color.r = (float) rng.uniform01();
	color.g = (float) rng.uniform01();
	color.b = (float) rng.uniform01();

	auto sphere_projections = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, default_qos());

	visualization_msgs::msg::MarkerArray projections_msg;

	visualization_msgs::msg::Marker balls;
	balls.header.frame_id = "world";
	balls.type = visualization_msgs::msg::Marker::POINTS;
	balls.color = color;
	balls.scale.x = 0.05;
	balls.scale.y = 0.05;
	balls.scale.z = 0.05;
	balls.pose.orientation.w = 1.0;

	visualization_msgs::msg::Marker lines;
	lines.header.frame_id = "world";
	lines.id = 1;
	lines.type = visualization_msgs::msg::Marker::LINE_LIST;
	lines.scale.x = 0.01;
	lines.color = color;
	lines.pose.orientation.w = 1.0;

	for (const auto& [apple, rs]: apples) {
		lines.points.push_back(pointEigenToMsg(apple.center));
		lines.points.push_back(pointEigenToMsg(rs.getGlobalLinkTransform("end_effector").translation()));
	}

	balls.points = lines.points;

	projections_msg.markers.push_back(balls);
	projections_msg.markers.push_back(lines);

	sphere_projections->publish(projections_msg);

	publisher_handles.push_back(sphere_projections);

}
//
//void ExperimentVisualTools::dumpApproaches(const std::shared_ptr<ompl::base::SpaceInformation> &si,
//										   const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
//										   const std::string &topic_name) {
//
//	ompl::geometric::PathGeometric combined_path(si);
//
//	for (const auto &approach: approaches) {
//		ompl::geometric::PathGeometric approach_copy(approach.second);
//		approach_copy.interpolate();
//		combined_path.append(approach_copy);
//	}
//
//	publishPath(si, topic_name, combined_path);
//
//}
//

void ExperimentVisualTools::publishPath(const std::shared_ptr<ompl::base::SpaceInformation> &si,
										const std::string &topic_name,
										const ompl::geometric::PathGeometric &combined_path) {

	auto moveit_trajectory = omplPathToRobotTrajectory(*si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>(),
													   combined_path);

	publishTrajectory(topic_name, moveit_trajectory);

}

moveit_msgs::msg::DisplayTrajectory robotTrajectoryToDisplayTrajectory(const robot_trajectory::RobotTrajectory &moveit_trajectory) {
	moveit_msgs::msg::DisplayTrajectory msg;
	msg.model_id = moveit_trajectory.getRobotModel()->getName();
	msg.trajectory.resize(1);

	moveit_trajectory.getRobotTrajectoryMsg(msg.trajectory[0]);
	moveit::core::robotStateToRobotStateMsg(moveit_trajectory.getFirstWayPoint(), msg.trajectory_start);
	return msg;
}

void ExperimentVisualTools::publishTrajectory(const std::string &topic_name,
											  const robot_trajectory::RobotTrajectory &moveit_trajectory) {

	moveit_msgs::msg::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(moveit_trajectory);

	auto traj = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(topic_name, default_qos());

	traj->publish(msg);

	publisher_handles.push_back(traj);
}
