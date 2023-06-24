// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../utilities/fcl_util.h"
#include "../utilities/experiment_utils.h"

#include <fcl/fcl.h>

int main(int argc, char **argv) {

	using namespace mgodpl::fcl_util;

	auto drone = loadRobotModel();

	auto tree_model = loadTreeMeshes("appletree");

	auto trunk = tree_model.trunk_mesh;

	auto trunk_collision = rosMeshMsgToCollisionObject(trunk);

	// Create FCL geometry for the box
	auto box = std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);

	fcl::CollisionObjectd boxCollisionObject(box);

	// Setup collision request and result structure
	fcl::ContinuousCollisionRequestd request;
	fcl::ContinuousCollisionResultd result;

	fcl::continuousCollide(
			&boxCollisionObject,
			Eigen::Isometry3d::Identity(),
			&trunk_collision,
			Eigen::Isometry3d::Identity(),
			request,
			result
			);

	if (result.is_collide) {
		std::cout << "Collision detected!" << std::endl;
	} else {
		std::cout << "No collision detected!" << std::endl;
	}

}