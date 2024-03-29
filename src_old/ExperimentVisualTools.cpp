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

//void ExperimentVisualTools::publishPath(const std::shared_ptr<ompl::base::SpaceInformation> &si,
//										const std::string &topic_name,
//										const ompl::geometric::PathGeometric &combined_path) {
//
//	auto moveit_trajectory = omplPathToRobotTrajectory(*si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>(),
//													   combined_path);
//
//	publishTrajectory(topic_name, moveit_trajectory);
//
//}

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

void ExperimentVisualTools::publishPath(const std::string &topic_name, const RobotPath &combined_path) {
	robot_trajectory::RobotTrajectory moveit_trajectory(combined_path.waypoints[0].getRobotModel());

	double t = 0.0;
	moveit::core::RobotState last_state = combined_path.waypoints[0];

	for (const auto &waypoint: combined_path.waypoints) {
		t += last_state.distance(waypoint);
		moveit_trajectory.addSuffixWayPoint(waypoint, t);
		last_state = waypoint;
	}

	publishTrajectory(topic_name, moveit_trajectory);

}

void ExperimentVisualTools::publishMesh(const shape_msgs::msg::Mesh &mesh, const std::string &topic_name) {

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
	mesh_marker.scale.x = 1.0;
	mesh_marker.scale.y = 1.0;
	mesh_marker.scale.z = 1.0;

	mesh_marker.pose.orientation.x = 0.0;
	mesh_marker.pose.orientation.y = 0.0;
	mesh_marker.pose.orientation.z = 0.0;
	mesh_marker.pose.orientation.w = 1.0;

	mesh_marker.color.r = 1.0;
	mesh_marker.color.g = 1.0;
	mesh_marker.color.b = 1.0;
	mesh_marker.color.a = 1.0;

	for (const auto &triangle: mesh.triangles) {
		mesh_marker.points.push_back(mesh.vertices[triangle.vertex_indices[0]]);
		mesh_marker.points.push_back(mesh.vertices[triangle.vertex_indices[2]]);
		mesh_marker.points.push_back(mesh.vertices[triangle.vertex_indices[1]]);
	}

	auto mesh_publisher = this->create_publisher<visualization_msgs::msg::Marker>(topic_name, default_qos());
	mesh_publisher->publish(mesh_marker);
	publisher_handles.push_back(mesh_publisher);

}
