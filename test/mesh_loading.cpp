
#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>
#include "../src/experiment_utils.h"
#include "../src/msgs_utilities.h"
#include "../src/general_utilities.h"



TEST(SceneTestRviz, test_rviz) {

    auto drone = loadRobotModel();

    auto [planning_scene_message,apples] = createMeshBasedAppleTreePlanningSceneMessage(appletree);

    int zero = 0;
    ros::init(zero,nullptr,"model_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    auto topic = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene_test", 10);

    while (topic.getNumSubscribers() == 0) ros::Duration(0.5).sleep();

    topic.publish(planning_scene_message);

    auto apples_topic = nh.advertise<visualization_msgs::MarkerArray>("/planning_scene_apple_positions", 10);
    while (apples_topic.getNumSubscribers() == 0) ros::Duration(0.5).sleep();

    visualization_msgs::MarkerArray ma;

    for (const auto &item : apples) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "apples";
        marker.id = ma.markers.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = item.center.x();
        marker.pose.position.y = item.center.y();
        marker.pose.position.z = item.center.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        ma.markers.push_back(marker);
    }

    apples_topic.publish(ma);

//    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
//    scene->getWorldNonConst()->mesh

//    scene->setPlanningSceneDiffMsg(createPlanningSceneDiff(treeFlattened, leafVertices, appleRadius, apples));
//
//    // Diff message apparently can't handle partial ACM updates?
//    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
//    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
//    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
//
//    return {apples, leafVertices, scene};

}