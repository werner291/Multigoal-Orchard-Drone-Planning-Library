
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <robowflex_library/builder.h>
#include "procedural_tree_generation.h"

Eigen::Isometry3d frame_on_branch(double azimuth, double t, const DetachedTreeNode &treeNode) {

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.rotate(Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ())); // TODO this is likely wrong.
    iso.rotate(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
    iso.translate(Eigen::Vector3d(0, t * treeNode.length, (float) treeNode.radius));
    iso = treeNode.root_at_absolute * iso;

    return iso;
}

void make_tree_branches(const Eigen::Isometry3d &root_at, unsigned int branching_depth, double root_radius,
                        std::vector<DetachedTreeNode> &nodes) {

    DetachedTreeNode node;
    node.root_at_absolute = root_at;
    node.length = 1.0;
    node.radius = root_radius;

    nodes.push_back(node);

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(-BRANCH_ANGULAR_RANGE, BRANCH_ANGULAR_RANGE);

    std::bernoulli_distribution split_probabilities(0.9);
    std::bernoulli_distribution split_double_probabilities(0.5);
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

            if (split_double_probabilities(eng) && branching_depth >= 2) {
                make_tree_branches(root_at * local_root,
                                   branching_depth - 2,
                                   child_radii[i] * pow(BRANCH_RADIUS_REDUCTION_FACTOR,2),
                                   nodes);

            } else {
                make_tree_branches(root_at * local_root,
                                   branching_depth - 1,
                                   child_radii[i] *  BRANCH_RADIUS_REDUCTION_FACTOR,
                                   nodes);
            }
        }
    }
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
            .global_xform = root.root_at
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
                    .global_xform = top.global_xform * child.root_at
            });
        }
    }

    return nodes;
}



std::vector<Apple> spawn_apples(const std::vector<DetachedTreeNode> &flattened,
                                const size_t NUMBER_OF_APPLES,
                                const double apple_radius) {

    std::vector<Apple> apples;

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(0.0,1.0);
    std::uniform_int_distribution<size_t> tree_branch_selection(1 /* No apples on the main trunk */, flattened.size() - 1);

    while (apples.size() < NUMBER_OF_APPLES) {
        size_t branch_index = tree_branch_selection(eng);

        const DetachedTreeNode &node = flattened[branch_index];

        auto apple_xform = frame_on_branch(distr(eng) * (float) M_PI * 2.0f, distr(eng), node) * Eigen::Translation3d(Eigen::Vector3d(0, 0, apple_radius));

            apples.push_back(Apple{
                    .center = apple_xform.translation(),
                    .branch_normal = (apple_xform.rotation() * Eigen::Vector3d(0.0,0.0,1.0))
            });
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
        if (branch_node.radius > 0.25) continue;

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

