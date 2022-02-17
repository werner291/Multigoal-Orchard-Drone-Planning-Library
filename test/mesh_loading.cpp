
#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include "../src/experiment_utils.h"

shape_msgs::Mesh meshMsgFromResource(const std::string &resource) {
    std::shared_ptr<shapes::Mesh> mesh_shape(shapes::createMeshFromResource(
            resource));
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh_shape.get(), mesh_msg);
    return boost::get<shape_msgs::Mesh>(mesh_msg);
}

void addColoredMeshCollisionShape(moveit_msgs::PlanningScene &planning_scene_message, const std::string &resource,
                                  const Eigen::Vector3f &rgb, const std::string &id) {

    moveit_msgs::CollisionObject tree_obj;
    tree_obj.id = id;
    tree_obj.header.frame_id = "world";
    tree_obj.meshes.push_back(meshMsgFromResource(resource));

    planning_scene_message.world.collision_objects.push_back(tree_obj);

    moveit_msgs::ObjectColor oc;
    oc.id = id;
    oc.color.r = rgb[0];
    oc.color.g = rgb[1];
    oc.color.b = rgb[2];
    oc.color.a = 1.0;
    planning_scene_message.object_colors.push_back(oc);
}

TEST(SceneTestRviz, test_rviz) {

    auto drone = loadRobotModel();

    moveit_msgs::PlanningScene planning_scene_message;

    addColoredMeshCollisionShape(planning_scene_message,
                                 "file:///home/werner/workspace/motion-planning-around-apple-trees/3d-models/appletree_trunk.dae",
                                 {0.5, 0.2, 0.1}, "trunk");
    addColoredMeshCollisionShape(planning_scene_message,
                                 "file:///home/werner/workspace/motion-planning-around-apple-trees/3d-models/appletree_apples.dae",
                                 {1.0, 0.0, 0.0}, "apples");
    addColoredMeshCollisionShape(planning_scene_message,
                                 "file:///home/werner/workspace/motion-planning-around-apple-trees/3d-models/appletree_leaves.dae",
                                 {0.1, 0.7, 0.1}, "leaves");


    int zero = 0;
    ros::init(zero,nullptr,"model_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    auto topic = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene_test", 10);

    while (topic.getNumSubscribers() == 0) ros::Duration(0.5).sleep();

    topic.publish(planning_scene_message);

//    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
//    scene->getWorldNonConst()->mesh



//
//
//    scene->setPlanningSceneDiffMsg(createPlanningSceneDiff(treeFlattened, leafVertices, appleRadius, apples));
//
//    // Diff message apparently can't handle partial ACM updates?
//    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
//    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
//    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
//
//    return {apples, leafVertices, scene};

}