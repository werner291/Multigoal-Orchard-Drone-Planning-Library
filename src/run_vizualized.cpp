#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include "build_request.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "MyCollisionDetectorAllocatorBullet.h"
#include "InverseClearanceIntegralObjective.h"
#include "ClearanceDecreaseMinimizationObjective.h"
#include <fcl/fcl.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection_fcl/collision_common.h>

using namespace robowflex;
//
//void constructFCLObjectRobot(const moveit::core::RobotState& state, collision_detection::FCLObject& fcl_obj) {
//
//    std::vector<collision_detection::FCLGeometryConstPtr> robot_geoms_;
//    std::vector<collision_detection::FCLObject> robot_fcl_objs_;
//    moveit::core::RobotModelConstPtr robot_model_ = state.getRobotModel();
//
//    const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
//    std::size_t index;
//    robot_geoms_.resize(robot_model_->getLinkGeometryCount());
//    robot_fcl_objs_.resize(robot_model_->getLinkGeometryCount());
//    // we keep the same order of objects as what RobotState *::getLinkState() returns
//    for (auto link : links)
//        for (std::size_t j = 0; j < link->getShapes().size(); ++j)
//        {
//            collision_detection::FCLGeometryConstPtr link_geometry = collision_detection::createCollisionGeometry(link->getShapes()[j], 1.0, 0.0, link, j);
//            if (link_geometry)
//            {
//                index = link->getFirstCollisionBodyTransformIndex() + j;
//                robot_geoms_[index] = link_geometry;
//
//                // Need to store the FCL object so the AABB does not get recreated every time.
//                // Every time this object is created, g->computeLocalAABB() is called  which is
//                // very expensive and should only be calculated once. To update the AABB, use the
//                // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
//                robot_fcl_objs_[index] =
//                        FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(link_geometry->collision_geometry_));
//            }
//            else
//                ROS_ERROR_NAMED(LOGNAME, "Unable to construct collision geometry for link '%s'", link->getName().c_str());
//        }
//
//    fcl_obj.collision_objects_.reserve(robot_geoms_.size());
//    fcl::Transform3d fcl_tf;
//
//    for (std::size_t i = 0; i < robot_geoms_.size(); ++i)
//        if (robot_geoms_[i] && robot_geoms_[i]->collision_geometry_) {
//            transform2fcl(state.getCollisionBodyTransform(robot_geoms_[i]->collision_geometry_data_->ptr.link,
//                                                          robot_geoms_[i]->collision_geometry_data_->shape_index),
//                          fcl_tf);
//            auto coll_obj = new fcl::CollisionObjectd(*robot_fcl_objs_[i]);
//            coll_obj->setTransform(fcl_tf);
//            coll_obj->computeAABB();
//            fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(coll_obj));
//        }
//}

class MyCollisionEnvironmentFCL : public collision_detection::CollisionEnvFCL {
public:
    MyCollisionEnvironmentFCL(const moveit::core::RobotModelConstPtr &model, double padding, double scale)
            : CollisionEnvFCL(model, padding, scale) {}

    void getRobotFCL(const moveit::core::RobotState& state, collision_detection::FCLObject& fcl_obj) {
        constructFCLObjectRobot(state, fcl_obj);
    }
};

/**
 * The "visualized" version of this program, which serves as a scratch state in which to experiment with new,
 * and potentially useless changes.
 *
 * See the benchmark main() method for the more reproducible results.
 */
int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene(10);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    auto optimizationObjectiveAllocator = [](const ompl::geometric::SimpleSetupPtr &ss) {
        return std::make_shared<ClearanceDecreaseMinimizationObjective>(ss->getSpaceInformation());
    };

        moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, "RRTConnect", 60.0,
                                                                       selectAppleNearCoG(tree_scene.apples));
        rviz.addGoalMarker("goal_request_marker", request);
        rviz.updateMarkers();
        auto simple_planner = init_planner(drone, scene, optimizationObjectiveAllocator);
        auto response = simple_planner->plan(scene, request);
        if (response.error_code_.val == 1) {


            std::cout << "Planning succeeded." << std::endl;



//        for (size_t i = 0; i < 10000; i++) {
//
//            response.trajectory_->getStateAtDurationFromStart(
//                    (double) i / 100.0 * response.trajectory_->getDuration(), drone->getScratchState());
//
//            drone->getScratchState()->update();
//
//            auto res = scene->checkCollision(*drone->getScratchState());
//
////            std::cout << "res:" << res.collision << std::endl;
//
////            fclEnv.getRobotFCL(*drone->getScratchState(), fclObject);
////
////            fcl::CollisionRequestd req;
////            fcl::CollisionResultd res;
////
////            fcl::collide(&leaves, Eigen::Isometry3d::Identity(),
////                         fclObject.collision_objects_[0]->collisionGeometry().get(),
////                         fclObject.collision_objects_[0]->getTransform(),
////                         req,
////                         res);
////
////            std::cout << "Collision: " << res.isCollision() << std::endl;
//        }
////        simple_planner->getLastSimpleSetup()->getPathSimplifier()->smoothBSpline()
//
            rviz.updateTrajectory(response);
        }

    std::cin.get();

    return 0;
}


