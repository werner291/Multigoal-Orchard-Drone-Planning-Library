// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-6-23.
//

#include "fcl_util.h"

fcl::BVHModel<fcl::OBBRSSd> mgodpl::fcl_util::rosMeshMsgToBvh(const shape_msgs::msg::Mesh &ros_mesh) {
	// create the BVH Model (BVH stands for Bounding Volume Hierarchy)
	fcl::BVHModel<fcl::OBBRSSd> bvh;

	bvh.beginModel();

	for (const auto &triangle: ros_mesh.triangles) {
		bvh.addTriangle(
				toEigen(ros_mesh.vertices[triangle.vertex_indices[0]]),
				toEigen(ros_mesh.vertices[triangle.vertex_indices[1]]),
				toEigen(ros_mesh.vertices[triangle.vertex_indices[2]])
				);
	}

	bvh.endModel();

	return bvh;
}

fcl::CollisionObject<double> mgodpl::fcl_util::rosMeshMsgToCollisionObject(const shape_msgs::msg::Mesh &ros_mesh) {
	// Convert the ROS Mesh to a FCL BVH Model
	fcl::BVHModel<fcl::OBBRSSd> bvhModel = rosMeshMsgToBvh(ros_mesh);

	// Wrap the BVH Model into a Collision Object with an identity transform
	fcl::CollisionObject<double> collisionObject(std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>(bvhModel), fcl::Transform3d::Identity());

	return collisionObject;
}
