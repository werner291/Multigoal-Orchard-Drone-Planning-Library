//
// Created by werner on 20-9-22.
//

#ifndef NEW_PLANNERS_APPROACHPLANNING_H
#define NEW_PLANNERS_APPROACHPLANNING_H

#include "../ExperimentVisualTools.h"
#include "../probe_retreat_move.h"
#include "../utilities/traveling_salesman.h"
#include "../utilities/general_utilities.h"
#include "../DronePathLengthObjective.h"
#include "../planning_scene_diff_message.h"
#include "../DistanceHeuristics.h"
#include "../shell_space/OmplShellSpace.h"
#include "MultiGoalPlanner.h"
#include <range/v3/view/transform.hpp>
#include <range/v3/view/enumerate.hpp>

template<typename ShellPoint>
struct ApproachPath {
	ShellPoint shell_point;
	RobotPath robot_path;
};
template<typename ShellPoint>
struct InitialApproachPath {
	ShellPoint shell_point;
	RobotPath robot_path;
};

template<typename ShellPoint>
class ApproachPlanningMethods {

public:

	virtual InitialApproachPath<ShellPoint> initial_approach_path(const ompl::base::State *start, const OmplShellSpace<>& shell) const = 0;

	virtual ApproachPath<ShellPoint> approach_path(const ompl::base::Goal* goal, const OmplShellSpace& shell) const = 0;

};

#endif //NEW_PLANNERS_APPROACHPLANNING_H
