//
// Created by werner on 9/2/21.
//

#include "MyOMPLPlanner.h"

planning_interface::MotionPlanResponse
MyOMPLPlanner::plan(const robowflex::SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) {
    return planning_interface::MotionPlanResponse();
}

std::vector<std::string> MyOMPLPlanner::getPlannerConfigs() const {
    return std::vector<std::string>();
}

MyOMPLPlanner::MyOMPLPlanner(const robowflex::RobotPtr &robot, const std::string &name,
                             const ompl::geometric::SimpleSetup &ss) : Planner(robot, name) {


}
