

#ifndef NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
#define NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H


#include <ompl/base/goals/GoalSampleableRegion.h>
#include "SphereShell.h"

class EndEffectorOnShellGoal : public ompl::base::GoalSampleableRegion {

    OMPLSphereShellWrapper sphereShell;
    Eigen::Vector3d focus;

public:
    EndEffectorOnShellGoal(const ompl::base::SpaceInformationPtr &si,
                           OMPLSphereShellWrapper sphereShell, Eigen::Vector3d focus);

    void sampleGoal(ompl::base::State *st) const override;

    [[nodiscard]] unsigned int maxSampleCount() const override;

    double distanceGoal(const ompl::base::State *st) const override;

};


#endif //NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
