
#include <random_numbers/random_numbers.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <Eigen/Geometry>
#include "procedural_tree_generation.h"
#include <stack>
#include <random>
#include <iostream>

Eigen::Isometry3d frame_on_branch(double azimuth, double t, const DetachedTreeNode &treeNode) {

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.rotate(Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ())); // TODO this is likely wrong.
    iso.rotate(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
    iso.translate(Eigen::Vector3d(0, t * treeNode.length, (float) treeNode.radius));
    iso = treeNode.root_at_absolute * iso;

    return iso;
}

TreeNode make_tree_branches(const Eigen::Isometry3d &root_at, unsigned int branching_depth, double root_radius) {

    TreeNode node;
    node.root_at_relative = root_at;
    node.length = 1.0;
    node.radius = root_radius;

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(-BRANCH_ANGULAR_RANGE, BRANCH_ANGULAR_RANGE);

    std::bernoulli_distribution split_probabilities(0.9);
    std::uniform_real_distribution<double> area_split_distr(0.3,0.7);

    if (branching_depth > 0) {

        int split_number = split_probabilities(eng) ? 2 : 1;

        double area_split = area_split_distr(eng);

        double parent_area = pow(root_radius, 2);

        double child_radii[2] = {
                sqrt(parent_area * area_split),
                sqrt(parent_area * (1.0- area_split)),
        };

        for (int i = 0; i < split_number; i++) {

            Eigen::Isometry3d local_root;
            local_root.setIdentity();
            local_root.translate(Eigen::Vector3d(0,0,node.length));
            // Pre-transform?
            local_root *= Eigen::AngleAxisd(distr(eng), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(distr(eng), Eigen::Vector3d::UnitY());

            node.children.push_back(make_tree_branches(local_root, branching_depth - 1, child_radii[i] *
                                                                                        BRANCH_RADIUS_REDUCTION_FACTOR));
        }
    }

    return node;
}

std::vector<DetachedTreeNode> flatten_tree(const TreeNode& root) {
    std::vector<DetachedTreeNode> nodes;

    struct StackFrame {
        const TreeNode& node;
        const Eigen::Isometry3d global_xform;
    };

    std::stack<StackFrame> visit_stack;
    visit_stack.push(StackFrame {
            .node = root,
            .global_xform = root.root_at_relative
    });

    while (!visit_stack.empty()) {

        StackFrame top = visit_stack.top();
        visit_stack.pop();

        DetachedTreeNode node = {
                .root_at_absolute =  top.global_xform,
                .length =  top.node.length,
                .radius =  top.node.radius
        };

        nodes.push_back(node);

        for (const auto &child : top.node.children) {
            visit_stack.push(StackFrame {
                    .node = child,
                    .global_xform = top.global_xform * child.root_at_relative
            });
        }
    }

    return nodes;
}



std::vector<Apple> spawn_apples(const std::vector<DetachedTreeNode> &flattened,
//                                fcl::DynamicAABBTreeCollisionManager &tree_model_broadphase,
                                const size_t NUMBER_OF_APPLES,
                                const double apple_radius) {

    std::vector<Apple> apples;

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(0.0,1.0);
    std::uniform_int_distribution<size_t> tree_branch_selection(1 /* No apples on the main trunk */, flattened.size() - 1);

    // Needs to be static because userdata implementation only takes a void pointer.
//    const auto static apple_userdata = APPLE;

    while (apples.size() < NUMBER_OF_APPLES) {
        size_t branch_index = tree_branch_selection(eng);

        const DetachedTreeNode &node = flattened[branch_index];

        auto apple_xform = frame_on_branch(distr(eng) * (float) M_PI * 2.0f, distr(eng), node) * Eigen::Translation3d(Eigen::Vector3d(0, 0, apple_radius));

//        const std::shared_ptr<fcl::Sphere> apple_shape = std::make_shared<fcl::Sphere>(apple_radius);
//
//        apple_shape->setUserData((void *) &apple_userdata);
//
//        auto apple_collision_object = std::make_unique<fcl::CollisionObject>(apple_shape, fcl::Transform3f(eigenToFCL(apple_xform)));
//
//        bool did_collide = false;
//
//        tree_model_broadphase.collide(apple_collision_object.get(), &did_collide, broadphase_collision_callback);
//
//        if (!did_collide) {

//            tree_model_broadphase.registerObject(apple_collision_object.get());

            apples.push_back(Apple{
                    .center = apple_xform.translation(),
                    .branch_normal = (apple_xform.rotation() * Eigen::Vector3d(0.0,0.0,1.0))
//                    .collision_object = std::move(apple_collision_object)
            });
//        }
    }

    return apples;
}

std::vector<Eigen::Vector3d> generateLeafVertices(std::vector<DetachedTreeNode> &flattened) {

    const double LEAF_X_SIZE = 0.1;
    const double LEAF_Y_SIZE = 0.2;

    Eigen::Vector3d leafTriangles[2][3] = {
            {
                    Eigen::Vector3d(0,0,0),
                    Eigen::Vector3d(-LEAF_X_SIZE,0,LEAF_Y_SIZE),
                    Eigen::Vector3d(LEAF_X_SIZE,0,LEAF_Y_SIZE),
            },
            {
                    Eigen::Vector3d(LEAF_X_SIZE,0,LEAF_Y_SIZE),
                    Eigen::Vector3d(-LEAF_X_SIZE,0,LEAF_Y_SIZE),
                    Eigen::Vector3d(0,0,2.0 * LEAF_Y_SIZE),
            }
    };

    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<double> azimuths(-M_PI, M_PI);
    std::uniform_real_distribution<double> t_distrib(0.0,1.0);
    std::uniform_real_distribution<double> leaf_rotated(-1.0,1.0);
    std::uniform_real_distribution<double> scale_range(0.5, 1.5);


    std::vector<Eigen::Vector3d> vertices;

    for (auto & branch_node : flattened) {
        if (branch_node.radius > 0.5) continue;

        for (size_t leaf_idx = 0; leaf_idx < NUM_LEAVES_PER_BRANCH; leaf_idx++) {

            Eigen::Affine3d frame = frame_on_branch(azimuths(eng), t_distrib(eng), branch_node) ;
            frame.rotate(Eigen::AngleAxisd(leaf_rotated(eng), Eigen::Vector3d::UnitY()));
            frame.scale(scale_range(eng));

            for (const auto &leafVertices : leafTriangles) {
                for (const auto & leafVertex : leafVertices) {
                    vertices.push_back(frame * leafVertex);
                }
            }

        }

    }
    return vertices;
}

PlanningScene establishPlanningScene() {

    TreeNode tree = make_tree_branches(Eigen::Isometry3d::Identity(), 6, 0.5);
    std::vector<DetachedTreeNode> tree_flattened = flatten_tree(tree);

    PlanningScene planning_scene{};

    {
        moveit_msgs::CollisionObject collision_object;

        collision_object.id = "trunk";
        collision_object.header.frame_id = "world";

        for (const auto &tree_node : tree_flattened) {
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = tree_node.radius;
            primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = tree_node.length;

            collision_object.primitives.push_back(primitive);

            geometry_msgs::Pose pose;

            Eigen::Vector3d position = tree_node.root_at_absolute.translation();
            Eigen::Quaterniond quat(tree_node.root_at_absolute.rotation());

            position += quat * Eigen::Vector3d(0, 0, tree_node.length / 2.0);

            pose.position.x = position.x();
            pose.position.x = position.x();
            pose.position.y = position.y();
            pose.position.y = position.y();
            pose.position.z = position.z();
            pose.position.z = position.z();

            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();

            collision_object.primitive_poses.push_back(pose);
        }

        planning_scene.moveit_diff.world.collision_objects.push_back(collision_object);

        moveit_msgs::ObjectColor col;
        col.id = "trunk";
        col.color.a = 1.0;
        col.color.r = 0.5;
        col.color.g = 0.3;
        col.color.b = 0.1;
        planning_scene.moveit_diff.object_colors.push_back(col);
    }

    {

        std::vector<Eigen::Vector3d> leaf_vertices = generateLeafVertices(tree_flattened);

        moveit_msgs::CollisionObject leaves_collision;
        leaves_collision.id = "leaves";
        leaves_collision.header.frame_id = "world";

        shape_msgs::Mesh mesh;
        for (const auto &vtx : leaf_vertices) {
            geometry_msgs::Point pt;
            pt.x = vtx.x();
            pt.y = vtx.y();
            pt.z = vtx.z();
            mesh.vertices.push_back(pt);
        }

        for (size_t i = 0; i < mesh.vertices.size();i+=3) {
            shape_msgs::MeshTriangle tri;
            tri.vertex_indices[0] = i;
            tri.vertex_indices[1] = i + 1;
            tri.vertex_indices[2] = i + 2;
            mesh.triangles.push_back(tri);
        }
        leaves_collision.meshes.push_back(mesh);
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        leaves_collision.mesh_poses.push_back(pose);

        //planning_scene.moveit_diff.world.collision_objects.push_back(leaves_collision);

        moveit_msgs::ObjectColor leaves_color;
        leaves_color.id = "leaves";
        leaves_color.color.a = 1.0;
        leaves_color.color.r = 0.0;
        leaves_color.color.g = 1.0;
        leaves_color.color.b = 0.0;
        //planning_scene.moveit_diff.object_colors.push_back(leaves_color);
    }

    {
        const double APPLE_RADIUS = 0.05;

        std::vector<Apple> apples = spawn_apples(tree_flattened, 50, APPLE_RADIUS);

        moveit_msgs::CollisionObject leaves_collision;
        leaves_collision.id = "apples";
        leaves_collision.header.frame_id = "world";

        for (const auto &apple : apples) {

            geometry_msgs::Pose pose;
            pose.position.x = apple.center.x();
            pose.position.y = apple.center.y();
            pose.position.z = apple.center.z();
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
            leaves_collision.primitive_poses.push_back(pose);


            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = APPLE_RADIUS;
            leaves_collision.primitives.push_back(primitive);
        }

        planning_scene.moveit_diff.world.collision_objects.push_back(leaves_collision);

        moveit_msgs::ObjectColor apples_color;
        apples_color.id = "apples";
        apples_color.color.a = 1.0;
        apples_color.color.r = 1.0;
        apples_color.color.g = 0.0;
        apples_color.color.b = 0.0;
        planning_scene.moveit_diff.object_colors.push_back(apples_color);

        planning_scene.apples = apples;
    }

    planning_scene.moveit_diff.is_diff = true;

    return planning_scene;
}
