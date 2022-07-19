#include <vector>

#include <geometric_shapes/shapes.h>
#include "LeavesCollisionChecker.h"

LeavesCollisionChecker::LeavesCollisionChecker(const std::vector <Eigen::Vector3d> &leaf_vertices) {

	assert(leaf_vertices.size() % 3 == 0);

	leaves.beginModel();

	// Add the leaf vertices to the model in steps of 3 vertices.
	for (size_t i = 0; i < leaf_vertices.size(); i += 3) {
		leaves.addTriangle(leaf_vertices[i], leaf_vertices[i + 1], leaf_vertices[i + 2]);
	}
	leaves.endModel();
}

/**
 * Create an FCL Collision Geometry for the given MoveIt/ROS shape
 * @param shape
 * @return
 */
std::shared_ptr <fcl::CollisionGeometryd> collisionGeometryFromShape(const shapes::ShapeConstPtr shape) {

	switch (shape->type) {

		case shapes::UNKNOWN_SHAPE:
			throw std::logic_error("Unknown shape.");
		case shapes::SPHERE:
			return std::make_shared<fcl::Sphered>(std::dynamic_pointer_cast<const shapes::Sphere>(shape)->radius);
		case shapes::CYLINDER:
			return std::make_shared<fcl::Cylinderd>(std::dynamic_pointer_cast<const shapes::Cylinder>(shape)->radius,
													std::dynamic_pointer_cast<const shapes::Cylinder>(shape)->length);
		case shapes::CONE:
			return std::make_shared<fcl::Coned>(std::dynamic_pointer_cast<const shapes::Cone>(shape)->radius,
												std::dynamic_pointer_cast<const shapes::Cone>(shape)->length);
		case shapes::BOX:
			return std::make_shared<fcl::Boxd>(std::dynamic_pointer_cast<const shapes::Box>(shape)->size[0],
											   std::dynamic_pointer_cast<const shapes::Box>(shape)->size[1],
											   std::dynamic_pointer_cast<const shapes::Box>(shape)->size[2]);
		case shapes::PLANE:
			return std::make_shared<fcl::Planed>(std::dynamic_pointer_cast<const shapes::Plane>(shape)->a,
												 std::dynamic_pointer_cast<const shapes::Plane>(shape)->b,
												 std::dynamic_pointer_cast<const shapes::Plane>(shape)->c,
												 std::dynamic_pointer_cast<const shapes::Plane>(shape)->d);
		default:
			throw std::logic_error("Not implemented");
	}

	throw std::logic_error("Control should not reach this point.");
}

std::set <size_t> LeavesCollisionChecker::checkLeafCollisions(moveit::core::RobotState &state) const {

	// Make sure the link transforms are up-to-date.
	state.updateCollisionBodyTransforms();

	// TODO Update this to take *all* links into account, not just the base link.
	const moveit::core::LinkModel *lm = state.getRobotModel()->getLinkModel("base_link");

	// Grab the total transform of the link in world space.
	Eigen::Isometry3d link_transform = state.getGlobalLinkTransform(lm);

	// Get the shapes and transforms of those shapes relative to the links.
	auto shape_transforms = lm->getCollisionOriginTransforms();
	auto shapes = lm->getShapes();

	// They should form pairs.
	assert(shape_transforms.size() == shapes.size());

	// Initialize an empty set of leaves in contacts.
	std::set<size_t> leaves_in_contact;

	// Check every shape in the link.
	for (size_t i = 0; i < shapes.size(); ++i) {

		// Extract FCL collision geometry. (TODO: Should perhaps be cached somehow?)
		auto collision_geometry = collisionGeometryFromShape(shapes[i]);

		// Transform to the global frame.
		Eigen::Isometry3d total_transform = link_transform * shape_transforms[i];

		// Check for collisions with the triangle soup of leaves.
		fcl::CollisionRequestd req(500, true);
		fcl::CollisionResultd res;
		fcl::collide(&leaves, Eigen::Isometry3d::Identity(), collision_geometry.get(), total_transform, req, res);

		// If there are collisions, add the leaf index to the set of leaves in contact.
		for (int i = 0; i < res.numContacts(); ++i) {
			auto item = res.getContact(i);
			leaves_in_contact.insert((item.o1 == &leaves) ? item.b1 : item.b2);
		}
	}

	// Return the set of leaves in contact.
	return leaves_in_contact;
}

ompl::base::Cost LeavesCollisionCountObjective::stateCost(const ompl::base::State *s) const {

	// Convert to a MoveIt state.
	moveit::core::RobotState st(this->robot);
	si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(st, s);

	// Cost is the number of unique leaves currently in collision.
	return ompl::base::Cost(this->leaves->checkLeafCollisions(st).size());
}

LeavesCollisionCountObjective::LeavesCollisionCountObjective(const ompl::base::SpaceInformationPtr &si,
															 const std::shared_ptr <moveit::core::RobotModel> &robot,
															 const std::shared_ptr <LeavesCollisionChecker> &leaves)
		: StateCostIntegralObjective(si, true), robot(robot), leaves(leaves) {
}
