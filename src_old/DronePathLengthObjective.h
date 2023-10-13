#ifndef NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H
#define NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

/**
 * Custom version of PathLengthOptimizationObjective that provides access to the custom informed state samplers:
 * InformedBetweenTwoDroneStatesSampler and InformedBetweenDroneStateAndTargetSampler.
 */
class DronePathLengthObjective : public ompl::base::PathLengthOptimizationObjective {
public:
	DronePathLengthObjective(const ompl::base::SpaceInformationPtr &si);

	/**
	 * Depending on the type of the goal, will create either a InformedBetweenTwoDroneStatesSampler or a InformedBetweenDroneStateAndTargetSampler.
	 *
	 * @param probDefn 			The ProblemDefinition to create the sampler for.
	 * @param maxNumberCalls 	The maximum number of calls to the sampler.
	 * @return 					The sampler.
	 */
	ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
															 unsigned int maxNumberCalls) const override;

};


#endif //NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H
