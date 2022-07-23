
#include <ompl/geometric/PathSimplifier.h>
#include <boost/range/irange.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "EndEffectorOnShellGoal.h"
#include "general_utilities.h"

ompl::geometric::PathGeometric optimize(const ompl::geometric::PathGeometric& path,
                                        const ompl::base::OptimizationObjectivePtr &objective,
                                        const std::shared_ptr<ompl::base::SpaceInformation> &si);

[[nodiscard]] ompl::geometric::PathGeometric optimizeExit(const ompl::base::Goal* goal,
                                                          const ompl::geometric::PathGeometric& path,
                                                          const ompl::base::OptimizationObjectivePtr &objective,
                                                          const MoveItAppleSphereShell &shell,
                                                          const std::shared_ptr<ompl::base::SpaceInformation> &si);

