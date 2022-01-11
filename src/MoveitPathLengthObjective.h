
#ifndef NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H
#define NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

class MoveitPathLengthObjective : public ompl::base::PathLengthOptimizationObjective {
public:
    MoveitPathLengthObjective(const ompl::base::SpaceInformationPtr &si);

    ompl::base::InformedSamplerPtr allocInformedStateSampler(
        const ompl::base::ProblemDefinitionPtr &probDefn,
        unsigned int maxNumberCalls) const override;

};


#endif //NEW_PLANNERS_MOVEITPATHLENGTHOBJECTIVE_H
