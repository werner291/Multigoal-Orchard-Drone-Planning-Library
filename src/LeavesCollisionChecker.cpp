//
// Created by werner on 09-09-21.
//

#include <vector>
#include <Eigen/Core>
#include <geometric_shapes/shapes.h>
#include "LeavesCollisionChecker.h"

LeavesCollisionChecker::LeavesCollisionChecker(const std::vector<Eigen::Vector3d> &leaf_vertices) {
    leaves.beginModel();
    for (size_t i = 0; i < leaf_vertices.size(); i += 3) {
        leaves.addTriangle(
                leaf_vertices[i],
                leaf_vertices[i + 1],
                leaf_vertices[i + 2]
        );
    }
    leaves.endModel();
}

std::shared_ptr<fcl::CollisionGeometryd> collisionGeometryFromShape(const shapes::ShapeConstPtr shape) {

    switch (shape->type) {

        case shapes::UNKNOWN_SHAPE:
            ROS_ERROR("Unknown shape.");
            return {nullptr};
        case shapes::SPHERE:
            return std::make_shared<fcl::Sphered>(
                    std::dynamic_pointer_cast<const shapes::Sphere>(shape)->radius);
        case shapes::CYLINDER:
            return std::make_shared<fcl::Cylinderd>(
                    std::dynamic_pointer_cast<const shapes::Cylinder>(shape)->radius,
                    std::dynamic_pointer_cast<const shapes::Cylinder>(shape)->length);
        case shapes::CONE:
            return std::make_shared<fcl::Coned>(
                    std::dynamic_pointer_cast<const shapes::Cone>(shape)->radius,
                    std::dynamic_pointer_cast<const shapes::Cone>(shape)->length);
        case shapes::BOX:
            return std::make_shared<fcl::Boxd>(
                    std::dynamic_pointer_cast<const shapes::Box>(shape)->size[0],
                    std::dynamic_pointer_cast<const shapes::Box>(shape)->size[1],
                    std::dynamic_pointer_cast<const shapes::Box>(shape)->size[2]);
        case shapes::PLANE:
            return std::make_shared<fcl::Planed>(
                    std::dynamic_pointer_cast<const shapes::Plane>(shape)->a,
                    std::dynamic_pointer_cast<const shapes::Plane>(shape)->b,
                    std::dynamic_pointer_cast<const shapes::Plane>(shape)->c,
                    std::dynamic_pointer_cast<const shapes::Plane>(shape)->d);
        case shapes::MESH:
            ROS_ERROR("Not implemented");
            return {nullptr};
        case shapes::OCTREE:
            ROS_ERROR("Not implemented");
            return {nullptr};
    }

    ROS_ERROR("Control should not reach this point.");
    return {nullptr};
}

std::set<size_t> LeavesCollisionChecker::checkLeafCollisions(moveit::core::RobotState &state) const {
    state.updateCollisionBodyTransforms();
    const moveit::core::LinkModel *lm = state.getRobotModel()->getLinkModel("base_link");
    Eigen::Isometry3d link_transform = state.getGlobalLinkTransform(lm);

    auto shape_transforms = lm->getCollisionOriginTransforms();
    auto shapes = lm->getShapes();

    assert(shape_transforms.size() == shapes.size());

    std::set<size_t> leaves_in_contact;

    for (size_t i = 0; i < shapes.size(); ++i) {

        auto collision_geometry = collisionGeometryFromShape(shapes[i]);

        Eigen::Isometry3d total_transform = link_transform * shape_transforms[i];

        fcl::CollisionRequestd req(500, true);
        fcl::CollisionResultd res;
        fcl::collide(&leaves, Eigen::Isometry3d::Identity(), collision_geometry.get(), total_transform, req, res);

        for (int i = 0; i < res.numContacts(); ++i) {
            auto item = res.getContact(i);
            leaves_in_contact.insert((item.o1 == &leaves) ? item.b1 : item.b2);
        }
    }

    return leaves_in_contact;
}

ompl::base::Cost LeavesCollisionCountObjective::stateCost(const ompl::base::State *s) const {

    moveit::core::RobotState st(this->robot);

    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(st, s);

    return ompl::base::Cost(this->leaves->checkLeafCollisions(st).size());
}

LeavesCollisionCountObjective::LeavesCollisionCountObjective(const ompl::base::SpaceInformationPtr &si,
                                                             const std::shared_ptr<moveit::core::RobotModel> &robot,
                                                             const std::shared_ptr<LeavesCollisionChecker> &leaves)
        : StateCostIntegralObjective(si, true), robot(robot), leaves(leaves) {}
