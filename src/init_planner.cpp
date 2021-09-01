//
// Created by werner on 8/25/21.
//

#include "BulletContinuousMotionValidator.h"
#include "ClearanceDecreaseMinimizationObjective.h"
#include "InverseClearanceIntegralObjective.h"
#include "EndEffectorConstraintSampler.h"
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include "init_planner.h"
#include "CostBiasedNearestNeighborPlanner.h"

std::shared_ptr<robowflex::OMPL::OMPLInterfacePlanner>
init_planner(const std::shared_ptr<robowflex::Robot> &drone, const std::shared_ptr<robowflex::Scene> &scene,
             ObjectiveFactory &allocateOptimizationObjective) {

    // We use the OMPLInterfacePlannerto be able to access the underlying OMPL planner directly.
    auto simple_planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(drone, "simple");

    robowflex::OMPL::Settings settings;
    settings.simplify_solutions = false;

    if (!simple_planner->initialize("test_robots/config/ompl_planning.yaml", settings)) {
        ROS_ERROR("Planner initialization failed.");
    }

//    simple_planner->getInterface().getPlanningContextManager()
//            .registerPlannerAllocator(
//                    "custom::BiCBNN",
//                    [&](const ompl::base::SpaceInformationPtr& si,
//                          const std::string& new_name,
//                          const ompl_interface::ModelBasedPlanningContextSpecification& spec)
//                          {
//
//                          return std::make_shared<CostBiasedNearestNeighborPlanner>(si, new_name, 0.05, 5.0);
//                    });

    simple_planner->getInterface()
            .getConstraintSamplerManager()
            .registerSamplerAllocator(std::make_shared<DroneStateConstraintSamplerAllocator>());

    simple_planner->setPrePlanCallback([drone, allocateOptimizationObjective](const ompl_interface::ModelBasedPlanningContextPtr &context, const robowflex::SceneConstPtr &scene,
                                           const planning_interface::MotionPlanRequest &request){
        const ompl::geometric::SimpleSetupPtr &ss = context->getOMPLSimpleSetup();

//        ss->setOptimizationObjective(
//                std::make_shared<ClearanceDecreaseMinimzationObjective>(ss->getSpaceInformation())
//        );
        ss->setOptimizationObjective(
                allocateOptimizationObjective(ss)
        );

        ss->getSpaceInformation()->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(ss->getSpaceInformation().get(), drone, scene));
    });

    return simple_planner;
}