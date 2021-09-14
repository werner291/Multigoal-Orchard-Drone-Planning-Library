
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include "procedural_tree_generation.h"

struct MultiGoalPlanResult {
    robowflex::Trajectory trajectory;
    Json::Value stats;
};

MultiGoalPlanResult plan_nn_rrtconnect(const std::vector<Apple> &apples,
                                       const moveit::core::RobotState &start_state,
                                       const robowflex::SceneConstPtr &scene,
                                       const robowflex::RobotConstPtr &robot);

MultiGoalPlanResult plan_random(const std::vector<Apple> &apples,
                                const moveit::core::RobotState &start_state,
                                const robowflex::SceneConstPtr &scene,
                                const robowflex::RobotConstPtr &robot);