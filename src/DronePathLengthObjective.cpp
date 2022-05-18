
#include "DronePathLengthObjective.h"
#include "InformedManipulatorDroneSampler.h"

DronePathLengthObjective::DronePathLengthObjective(const ompl::base::SpaceInformationPtr &si)
        : PathLengthOptimizationObjective(si) {



        }

ompl::base::InformedSamplerPtr DronePathLengthObjective::allocInformedStateSampler(
     const ompl::base::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
 {
     return
        std::make_shared<InformedManipulatorDroneSampler>(probDefn, maxNumberCalls);
 }
