// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-6-23.
//

#ifndef MGODPL_FCL_UTIL_H
#define MGODPL_FCL_UTIL_H

#include <memory>

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <shape_msgs/msg/mesh.hpp>
#include "msgs_utilities.h"

namespace mgodpl::fcl_util {

	/**
	 * @brief Converts a ROS Mesh to a FCL BVH Model
	 *
	 * This function converts a ROS Mesh (shape_msgs::msg::Mesh) to a
	 * FCL (Flexible Collision Library) BVH Model (fcl::BVHModel).
	 *
	 * The function uses the OBBRSSd model in FCL which stands for Oriented Bounding Box
	 * with the properties of both RSS (Rectangular Solid Swept Sphere) and OBB (Oriented Bounding Box).
	 * This bounding volume type is efficient for a wide range of collision detection problems,
	 * which makes it a good general-purpose choice.
	 *
	 * @param ros_mesh The ROS Mesh to convert.
	 * @return The FCL BVH Model in OBBRSSd type.
	 */
	fcl::BVHModel<fcl::OBBRSSd> rosMeshMsgToBvh(const shape_msgs::msg::Mesh &ros_mesh);

	/**
	 * @brief Converts a ROS Mesh to a FCL Collision Object
	 *
	 * This function converts a ROS Mesh (shape_msgs::msg::Mesh) to a
	 * FCL (Flexible Collision Library) Collision Object (fcl::CollisionObject<double>).
	 *
	 * The function uses the OBBRSSd model in FCL which stands for Oriented Bounding Box
	 * with the properties of both RSS (Rectangular Solid Swept Sphere) and OBB (Oriented Bounding Box).
	 * This bounding volume type is efficient for a wide range of collision detection problems,
	 * which makes it a good general-purpose choice.
	 *
	 * The created FCL Collision Object has an identity transform.
	 *
	 * @param ros_mesh The ROS Mesh to convert.
	 * @return The FCL Collision Object with an identity transform.
	 */
	fcl::CollisionObject<double> rosMeshMsgToCollisionObject(const shape_msgs::msg::Mesh &ros_mesh);
}

#endif //MGODPL_FCL_UTIL_H
