//
// Created by werner on 9/2/21.
//

#ifndef NEW_PLANNERS_MYOMPLPLANNER_H
#define NEW_PLANNERS_MYOMPLPLANNER_H

#include <robowflex_library/planning.h>
#include <ompl/geometric/SimpleSetup.h>

class MyOMPLPlanner : public robowflex::Planner {

    ompl::base::SpaceInformationPtr si_;
public:
    MyOMPLPlanner(const robowflex::RobotPtr &robot, const std::string &name, const ompl::geometric::SimpleSetup &ss);

public:
    planning_interface::MotionPlanResponse
    plan(const robowflex::SceneConstPtr &scene, const planning_interface::MotionPlanRequest &request) override;

    std::vector<std::string> getPlannerConfigs() const override;

};


#endif //NEW_PLANNERS_MYOMPLPLANNER_H
