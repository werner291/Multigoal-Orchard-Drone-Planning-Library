
#include "ManipulatorDroneMoveitPathLengthObjective.h"
#include "InformedManipulatorDroneSampler.h"

ManipulatorDroneMoveitPathLengthObjective::ManipulatorDroneMoveitPathLengthObjective(const ompl::base::SpaceInformationPtr &si)
        : PathLengthOptimizationObjective(si) {



        }

ompl::base::InformedSamplerPtr ManipulatorDroneMoveitPathLengthObjective::allocInformedStateSampler(
     const ompl::base::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
 {
     return
        std::make_shared<InformedManipulatorDroneSampler>(probDefn, maxNumberCalls);
 }
