// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <chrono>
#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/enumerate.hpp>

#include "../experiment_utils/TreeMeshes.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../experiment_utils/load_robot_model.h"
#include "../planning/CollisionDetection.h"
#include "../planning/JointSpacePoint.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/VtkRobotModel.h"

using namespace mgodpl;
using namespace tree_meshes;
//using namespace visualization;
using namespace math;
using namespace moveit_facade;

void check_collision_custom(const moveit::core::RobotModelPtr& robot, fcl::CollisionObjectd tco, JointSpacePoint jt, bool& collides, fcl::Vector3d& contact_point)
{
    moveit::core::RobotState state(robot);
    jt.to_moveit(state);
    state.updateCollisionBodyTransforms();

    collides = false;

    for (const auto& link : robot->getLinkModelsWithCollisionGeometry())
    {

        const auto& box = std::dynamic_pointer_cast<const shapes::Box>(link->getShapes().front());

        const auto& tf = state.getCollisionBodyTransform(link, 0);

        fcl::CollisionObjectd co(std::make_shared<fcl::Boxd>(box->size[0], box->size[1], box->size[2]), tf);

        fcl::CollisionRequestd req;
        req.enable_contact = true;
        fcl::CollisionResultd res;

        collide(&tco, &co, req, res);

        if (res.isCollision())
        {
            contact_point = res.getContact(0).pos;
            collides = true;
            break;
        }
    }
}

std::vector<int> mksamples(const moveit::core::RobotModelPtr& robot, const shape_msgs::msg::Mesh& shape,
                           const std::vector<math::Vec3d>& fruit, const size_t N_SAMPLES)
{
    std::vector<int> success_counts;
    success_counts.reserve(fruit.size());

    random_numbers::RandomNumberGenerator rng(42);

    CollisionDetection cd({shape}, robot);

    auto g = std::make_shared<fcl::BVHModel<fcl::OBBd>>();

    std::cout << "N triangles: " << shape.triangles.size() << std::endl;

    std::vector<fcl::Triangle> tri_indices(shape.triangles.size());
    for (unsigned int i = 0; i < shape.triangles.size(); ++i)
    {
        tri_indices[i] =
            fcl::Triangle(
                shape.triangles[i].vertex_indices[0],
                shape.triangles[i].vertex_indices[1],
                shape.triangles[i].vertex_indices[2]);
    }

    std::vector<fcl::Vector3d> points(shape.vertices.size());
    for (unsigned int i = 0; i < shape.vertices.size(); ++i)
        points[i] = fcl::Vector3d(shape.vertices[i].x, shape.vertices[i].y, shape.vertices[i].z);

    g->beginModel();
    g->addSubModel(points, tri_indices);
    g->endModel();

    // Make the collision object with identity transform.
    fcl::CollisionObjectd tco(g, fcl::Transform3d::Identity());

    for (const auto& f : fruit)
    {
        int successes = 0;

        auto time_before = std::chrono::high_resolution_clock::now();

        for (int sample_i = 0; sample_i < N_SAMPLES; ++sample_i)
        {
            JointSpacePoint jt = experiment_state_tools::genGoalSampleUniform(f, rng, *robot);

            bool collides;
            fcl::Vector3d contact_point;
            check_collision_custom(robot, tco, jt, collides, contact_point);

            // bool collides_old = cd.collides(jt);
            //
            // if (collides_old != collides)
            // {
            //     std::cout << "Collision detection disagrees with FCL!" << std::endl;
            //     std::cout << "Moveit says: " << cd.collides(jt) << std::endl;
            //     std::cout << "We say: " << collides << std::endl;
            //
            //     SimpleVtkViewer viewer;
            //     viewer.addMesh(shape, {0.5, 0.3, 0.1});
            //
            //     mkPointMarkerSphere(Vec3d(contact_point.data()), viewer);
            //
            //     visualization::VtkRobotModel robotModelViz(
            //         robot, jt, !collides ? Vec3d(0.5, 0.5, 0.5) : Vec3d(0.5, 0.0, 0.0));
            //
            //
            //     viewer.addActorCollection(robotModelViz.getLinkActors());
            //     viewer.start();
            //
            //     abort();
            // }

            if (!collides)
            {
                ++successes;
            }
        }

        std::cout << "Taking " << N_SAMPLES << " samples for fruit " << f << " took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - time_before).count()
            << " ms." << std::endl;

        success_counts.push_back(successes);
    }

    return success_counts;
}

int main(int argc, char** argv)
{
    const auto& robot = experiment_assets::loadRobotModel(1.0);

    const auto& treemodel = loadTreeMeshes("appletree");

    // CollisionDetection cd({treemodel.trunk_mesh}, robot);

    const auto& fruit = computeFruitPositions(treemodel);

    const size_t N_SAMPLES = 1000;

    // SimpleVtkViewer viewer;
    // viewer.addMesh(treemodel.trunk_mesh, {0.5, 0.3, 0.1});
    //
    // viewer.lockCameraUp();
    Vec3d target(0, 0, 2);

    std::vector<int> success_counts = mksamples(robot, treemodel.trunk_mesh, fruit, N_SAMPLES);

    for (const auto& [i, f] : ranges::views::enumerate(fruit))
    {
        std::cout << "Fruit " << i << " has " << success_counts[i] << "/" << N_SAMPLES << " successes." << std::endl;
    }

    //
    // std::cout << "Target with least successes: " << least_successes_fruit << " with " << least_successes << " successes." << std::endl;
    //
    // // For the sample with the least successes, visualize the robot.
    // for (int sample_i = 0; sample_i < 100; ++sample_i) {
    // 	JointSpacePoint jt = experiment_state_tools::genGoalSampleUniform(least_successes_fruit, sample_i, *robot);
    //
    //
    // 		visualization::VtkRobotModel robotModelViz(robot, jt, !cd.collides(jt) ? Vec3d(0.5, 0.5, 0.5) : Vec3d(0.5, 0.0, 0.0));
    //
    // 		viewer.addActorCollection(robotModelViz.getLinkActors());
    // }
    //
    // viewer.setCameraTransform(
    // 	least_successes_fruit + Vec3d(1, 0, 0),
    // 	least_successes_fruit);

    // viewer.start();
}
