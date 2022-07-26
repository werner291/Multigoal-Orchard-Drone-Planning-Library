
#ifndef NEW_PLANNERS_CONVEXHULLSHELL_H
#define NEW_PLANNERS_CONVEXHULLSHELL_H

#include "SphereShell.h"
#include "planners/ShellPathPlanner.h"

class ConvexHullShellBuilder : public ShellPathPlanner::ShellBuilder {

public:
	std::shared_ptr<OMPLSphereShellWrapper>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) override;

	Json::Value parameters() const override;

};

class ConvexHullShell : public CollisionFreeShell {

public:
	moveit::core::RobotState
		state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const override;

	std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
														const Eigen::Vector3d &a,
														const Eigen::Vector3d &b) const override;

	double predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const override;

protected:
	Eigen::Vector3d project(const Eigen::Vector3d &a) const override;

};


#endif //NEW_PLANNERS_CONVEXHULLSHELL_H
