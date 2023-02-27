// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-2-23.
//

#ifndef NEW_PLANNERS_ALPHA_SHAPE_H
#define NEW_PLANNERS_ALPHA_SHAPE_H

#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>

const double LEAVES_ALPHA_SQRTRADIUS = sqrt(0.0001);

/**
 * @brief Compute the alpha shape of a set of points.
 *
 * @param points 	The points to compute the alpha shape of.
 * @param alpha 	The square root of the empty sphere radius (following CGAL's convention).
 * @return 			A shapes::Mesh object representing the alpha shape.
 */
shape_msgs::msg::Mesh alphaShape(const std::vector<Eigen::Vector3d> &points, double alpha = LEAVES_ALPHA_SQRTRADIUS);

#endif //NEW_PLANNERS_ALPHA_SHAPE_H
