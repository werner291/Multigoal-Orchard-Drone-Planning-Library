// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/20/24.
//

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Kernel/Type_equality_wrapper.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

#include "swept_volume_ccd.h"

// Compute the convex hull using CGAL.
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Polyhedron_3 = CGAL::Polyhedron_3<K>;
using Point_3 = K::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

std::vector<std::array<mgodpl::math::Vec3d, 3>>
mgodpl::swept_volume_triangles(const mgodpl::robot_model::RobotModel &robot,
							   const mgodpl::RobotState &state1,
							   const mgodpl::RobotState &state2,
							   size_t segments) {

	std::vector<std::array<math::Vec3d, 3>> triangles;

	for (size_t i = 0; i < segments; ++i) {
		double t1 = i / (double) segments;
		double t2 = (i + 1) / (double) segments;

		auto st1 = interpolate(state1, state2, t1);
		auto st2 = interpolate(state1, state2, t2);

		auto fk1 = forwardKinematics(robot, st1.joint_values, 0, st1.base_tf);
		auto fk2 = forwardKinematics(robot, st2.joint_values, 0, st2.base_tf);

		// For every link:
		for (size_t i = 0; i < robot.getLinks().size(); ++i) {

			for (const auto &shape: robot.getLinks()[i].collision_geometry) {
				auto box = std::get<Box>(shape.shape);

				auto tf_before = fk1.forLink(i);
				auto tf_after = fk2.forLink(i);

				// Compute the convex hull of the two boxes.
				std::vector<math::Vec3d> points;
				points.push_back(tf_before.apply(math::Vec3d{box.size.x() / 2, box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{box.size.x() / 2, box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{box.size.x() / 2, -box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{box.size.x() / 2, -box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{-box.size.x() / 2, box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{-box.size.x() / 2, box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{-box.size.x() / 2, -box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_before.apply(math::Vec3d{-box.size.x() / 2, -box.size.y() / 2, -box.size.z() / 2}));

				points.push_back(tf_after.apply(math::Vec3d{box.size.x() / 2, box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{box.size.x() / 2, box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{box.size.x() / 2, -box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{box.size.x() / 2, -box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{-box.size.x() / 2, box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{-box.size.x() / 2, box.size.y() / 2, -box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{-box.size.x() / 2, -box.size.y() / 2, box.size.z() / 2}));
				points.push_back(tf_after.apply(math::Vec3d{-box.size.x() / 2, -box.size.y() / 2, -box.size.z() / 2}));

				std::vector<K::Point_3> cgal_points;
				for (const auto &point: points) {
					cgal_points.emplace_back(point.x(), point.y(), point.z());
				}

				// define polyhedron to hold convex hull
				Polyhedron_3 poly;
				// compute convex hull of non-collinear points
				CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

				// Extract the triangles from the polyhedron.
				for (auto f = poly.facets_begin(); f != poly.facets_end(); ++f) {
					auto h = f->halfedge();
					auto p0 = h->vertex()->point();
					auto p1 = h->next()->vertex()->point();
					auto p2 = h->next()->next()->vertex()->point();
					triangles.push_back({
												math::Vec3d(p0.x(), p0.y(), p0.z()),
												math::Vec3d(p1.x(), p1.y(), p1.z()),
												math::Vec3d(p2.x(), p2.y(), p2.z())
										});
				}

			}

		}

	}
	return triangles;
}
