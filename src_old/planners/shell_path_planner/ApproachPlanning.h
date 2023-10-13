
#ifndef NEW_PLANNERS_APPROACHPLANNING_H
#define NEW_PLANNERS_APPROACHPLANNING_H

#include <optional>
#include <memory>
#include <ompl/geometric/PathGeometric.h>
#include "../../shell_space/OmplShellSpace.h"

template<typename ShellPoint>
struct OmplApproachPath {
	ShellPoint shell_point;
	ompl::geometric::PathGeometric robot_path;
};

template<typename ShellPoint>
class ApproachPlanningMethods {

public:

	virtual std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::State *start, const OmplShellSpace<ShellPoint> &shell) const = 0;

	virtual std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<ShellPoint> &shell) const = 0;

	virtual ~ApproachPlanningMethods() = default;
};


#endif //NEW_PLANNERS_APPROACHPLANNING_H
