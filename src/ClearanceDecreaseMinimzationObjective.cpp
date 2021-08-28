#include <robowflex_ompl/ompl_interface.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include "ClearanceDecreaseMinimzationObjective.h"
#include "MyCollisionDetectorAllocatorBullet.h"

//double fullBodyClearance(moveit::core::RobotState state, const planning_scene::PlanningScene& scene) {
//    state.update();
//
//    auto env = std::dynamic_pointer_cast<const MyCollisionEnvironment>(scene.getCollisionEnv());
//
//
//
////    env->getWorld()
//
////    scene.checkCollision()
//
//    return 0.0;
//}

ompl::base::Cost ClearanceDecreaseMinimzationObjective::stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
}

ompl::base::Cost ClearanceDecreaseMinimzationObjective::identityCost() const {
    return ompl::base::Cost(0.0);
}

ompl::base::Cost ClearanceDecreaseMinimzationObjective::infiniteCost() const {
    return ompl::base::Cost(std::numeric_limits<double>::infinity());
}

bool ClearanceDecreaseMinimzationObjective::isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const {
    return c1.value() < c2.value();
}
