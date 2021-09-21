
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include "procedural_tree_generation.h"


struct MultiGoalPlanResult {
    robowflex::Trajectory trajectory;
    Json::Value stats;
};

struct PointToPointPlanResult {
    std::optional<double> solution_length;
};

MultiGoalPlanResult plan_nn(const std::vector<Apple> &apples,
                            const moveit::core::RobotState &start_state,
                            const robowflex::SceneConstPtr &scene,
                            const robowflex::RobotConstPtr &robot,
                            ompl::base::Planner &point_to_point_planner);

MultiGoalPlanResult plan_knn(const std::vector<Apple> &apples,
                            const moveit::core::RobotState &start_state,
                            const robowflex::SceneConstPtr &scene,
                            const robowflex::RobotConstPtr &robot,
                            size_t k,
                            ompl::base::Planner &point_to_point_planner);

MultiGoalPlanResult plan_random(const std::vector<Apple> &apples,
                                const moveit::core::RobotState &start_state,
                                const robowflex::SceneConstPtr &scene,
                                const robowflex::RobotConstPtr &robot,
                                ompl::base::Planner &point_to_point_planner);

MultiGoalPlanResult plan_k_random(const std::vector<Apple> &apples,
                                  const moveit::core::RobotState &start_state,
                                  const robowflex::SceneConstPtr &scene,
                                  const robowflex::RobotConstPtr &robot,
                                  size_t k,
                                  ompl::base::Planner &point_to_point_planner);

PointToPointPlanResult planPointToPoint(const robowflex::RobotConstPtr &robot, robowflex::Trajectory &full_trajectory,
                                        ompl::base::Planner &planner, const ompl::base::GoalPtr &goal);