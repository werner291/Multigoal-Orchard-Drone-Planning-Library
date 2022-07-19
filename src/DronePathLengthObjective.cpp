#include <ompl/base/goals/GoalState.h>
#include "DronePathLengthObjective.h"
#include "InformedBetweenTwoDroneStatesSampler.h"

DronePathLengthObjective::DronePathLengthObjective(const ompl::base::SpaceInformationPtr &si)
		: PathLengthOptimizationObjective(si) {
}

ompl::base::InformedSamplerPtr
DronePathLengthObjective::allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
													unsigned int maxNumberCalls) const {

	// Use the null-pointer-returning behavior of dynamic casting to
	// tell whether the goal is a single state or something else.
	if (std::dynamic_pointer_cast<ompl::base::GoalState>(probDefn->getGoal())) {
		return std::make_shared<InformedBetweenTwoDroneStatesSampler>(probDefn, maxNumberCalls);
	} else {
		return std::make_shared<InformedBetweenDroneStateAndTargetSampler>(probDefn, maxNumberCalls);
	}

}
