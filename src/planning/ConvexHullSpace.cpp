// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8-11-23.
//

#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "ConvexHullSpace.h"

namespace mgodpl {

	using namespace cgal;

	Surface_mesh make_chull(const std::vector<math::Vec3d> &points) {
		Surface_mesh mesh;

		std::vector<Point_3> cgal_points;

		// Add all the leaf vertices.
		for (const auto &point: points) {
			cgal_points.emplace_back(point.x(), point.y(), point.z());
		}

		// Grab the convex hull.
		CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), mesh);

		return mesh;
	}

	ConvexHullShellSpace::ConvexHullShellSpace(const std::vector<math::Vec3d> &points) :
											   mesh(make_chull(points)),
											   mesh_path(mesh) {
		mesh_path.build_aabb_tree(tree);
	}

	ConvexHullShellSpace::ShellPoint ConvexHullShellSpace::closestPoint(const math::Vec3d &point) const {
		return mesh_path.locate(Point_3(point.x(), point.y(), point.z()), tree);
	}

	ConvexHullShellSpace::CarthesianShellPoint ConvexHullShellSpace::shell_state(const ConvexHullShellSpace::ShellPoint &sp) const {
		const auto &pt = mesh_path.point(sp.first, sp.second);
		const auto &nm = CGAL::Polygon_mesh_processing::compute_face_normal(sp.first, mesh);

		math::Vec3d pt_vec(pt.x(), pt.y(), pt.z());
		math::Vec3d nm_vec(nm.x(), nm.y(), nm.z());

		return {
			.normal = nm_vec,
			.point = pt_vec
		};
	}

	std::vector<double> ConvexHullShellSpace::distances_to_many(const ConvexHullShellSpace::ShellPoint &sp,
																const std::vector<ShellPoint> &other_points) const {
		Surface_mesh_shortest_path other_mesh_path(mesh);
		other_mesh_path.add_source_point(sp.first, sp.second);

		return other_points | ranges::views::transform([&](const auto &other_sp) {
			return other_mesh_path.shortest_distance_to_source_points(other_sp.first, other_sp.second).first;
		}) | ranges::to<std::vector>();
	}

	std::vector<ConvexHullShellSpace::ShellPoint>
	ConvexHullShellSpace::path_along_shell(const ConvexHullShellSpace::ShellPoint &start,
										   const ConvexHullShellSpace::ShellPoint &end) const {

		Surface_mesh_shortest_path other_mesh_path(mesh); // TODO: this rebuilds the sequence tree; that might not be necessary? Can we cache the last source point?

		other_mesh_path.add_source_point(start.first, start.second);

		std::vector<ShellPoint> path;

		PathVisitor path_visitor{.mesh = mesh, .path_algo = other_mesh_path, .states = path};

		other_mesh_path.shortest_path_sequence_to_source_points(end.first, end.second, path_visitor);

		std::reverse(path.begin(), path.end());

		return path;

	}

	math::Vec3d ConvexHullShellSpace::to_carthesian(const ConvexHullShellSpace::ShellPoint &sp) const {
		const auto &pt = mesh_path.point(sp.first, sp.second);
		return math::Vec3d(pt.x(), pt.y(), pt.z());
	}
}
