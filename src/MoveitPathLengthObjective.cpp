
#include "MoveitPathLengthObjective.h"
#include "BetweenMoveItStatesInformedSampler.h"

MoveitPathLengthObjective::MoveitPathLengthObjective(const ompl::base::SpaceInformationPtr &si)
        : PathLengthOptimizationObjective(si) {



        }

ompl::base::InformedSamplerPtr MoveitPathLengthObjective::allocInformedStateSampler(
     const ompl::base::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
 {
     return
        std::make_shared<BetweenMoveItStatesInformedSampler>(probDefn, maxNumberCalls);
 }
