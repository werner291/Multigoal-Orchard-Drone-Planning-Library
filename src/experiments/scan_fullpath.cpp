// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../visualization/visualization_function_macros.h"

import visualization.ThrottledRunQueue;

#include <ranges>

#include "../experiment_utils/TreeMeshes.h"
#include "../planning/cgal_chull_shortest_paths.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../math/Segment3d.h"
#include "../planning/scannable_points.h"
#include "../planning/traveling_salesman.h"
#include "../visualization/VtkPolyLineVisualization.h"

namespace mgodpl::math {
	struct Segment3d;
}

using namespace mgodpl;

using ShellSpace = cgal::CgalMeshData;

cgal::CgalMeshData compute_convex_hull_of_crown(const tree_meshes::TreeMeshes &tree_model) {
	// First, create the convex hull.
	cgal::CgalMeshData mesh_data(tree_model.leaves_mesh);
	return mesh_data;
}

auto visualize(SimpleVtkViewer &viewer, const ShellSpace &mesh_data) {
	Mesh mesh = cgal::cgalMeshToMesh(mesh_data.convex_hull);
	return viewer.addMesh(mesh, {0.8, 0.8, 0.8}, 0.5);
}

auto visualize(SimpleVtkViewer &viewer, const tree_meshes::TreeMeshes &tree_model) {
	return viewer.addTree(tree_model);
}

struct RGBColor {
	double r;
	double g;
	double b;
};

auto visualize(SimpleVtkViewer &viewer, std::ranges::range auto &segments, RGBColor color) {
	VtkLineSegmentsVisualization viz(color.r, color.g, color.b);

	std::vector<std::pair<math::Vec3d, math::Vec3d> > lines;
	for (const auto &segment: segments) {
		lines.push_back({segment.a, segment.b});
	}

	viz.updateLine(lines);

	viewer.addActor(viz.getActor());

	return viz;
}

auto project_to_shell(const ShellSpace &shell, const math::Vec3d &pt) {
	return cgal::locate_nearest(pt, shell);
}

auto to_euclidean(const ShellSpace &shell, const cgal::Surface_mesh_shortest_path::Face_location &fl) {
	return cgal::from_face_location(fl, shell);
}

using ShellPoint = cgal::Surface_mesh_shortest_path::Face_location;

struct GeodesicPath {
	std::vector<ShellPoint> path;
};

auto geodesics_one_to_many(const ShellSpace &shell,
                           const ShellPoint origin,
                           std::ranges::range auto &targets) {
	cgal::Surface_mesh_shortest_path mesh_path(shell.convex_hull);
	mesh_path.add_source_point(origin.first, origin.second);

	std::vector<GeodesicPath> paths;
	std::vector<ShellPoint> path;

	cgal::PathVisitor path_visitor{.mesh = shell.convex_hull, .path_algo = mesh_path, .states = path};

	for (const auto &target: targets) {
		mesh_path.shortest_path_sequence_to_source_points(target.first, target.second, path_visitor);
		paths.push_back({path});
		path.clear();
	}

	return paths;
}

auto geodesic_one_to_one(const ShellSpace &shell,
                         const ShellPoint &origin,
                         const ShellPoint &target) {
	cgal::Surface_mesh_shortest_path mesh_path(shell.convex_hull);
	mesh_path.add_source_point(origin.first, origin.second);

	std::vector<ShellPoint> path;

	cgal::PathVisitor path_visitor{.mesh = shell.convex_hull, .path_algo = mesh_path, .states = path};

	mesh_path.shortest_path_sequence_to_source_points(target.first, target.second, path_visitor);

	return GeodesicPath{path};
}

using GeodesicSingleHook = std::function<void(const ShellPoint &, const cgal::Surface_mesh_shortest_path &)>;

auto geodesic_distance_one_to_many(const ShellSpace &shell,
                                   const ShellPoint origin,
                                   std::ranges::range auto &targets) {
	cgal::Surface_mesh_shortest_path mesh_path(shell.convex_hull);
	mesh_path.add_source_point(origin.first, origin.second);

	std::vector<double> distances;

	for (const auto &target: targets) {
		distances.push_back(mesh_path.shortest_distance_to_source_points(target.first, target.second).first);
	}

	return distances;
}

struct DistanceMatrix {
	std::vector<std::vector<double> > distances;
};

using DistanceMatrixHook = std::function<void(const ShellPoint &)>;

auto geodesic_distance_many_to_many(const ShellSpace &shell,
                                    const ShellPoint origin,
                                    std::ranges::range auto &targets,
                                    const std::optional<DistanceMatrixHook> &hook) {
	// Compute the n^2 matrix of distances:
	DistanceMatrix matrix;

	for (const auto &target: targets) {
		if (hook.has_value()) {
			hook.value()(target);
		}

		matrix.distances.push_back(
			geodesic_distance_one_to_many(shell, origin, targets));
	}

	return matrix;
}

/**
 * Compute an optimal permutation of the targets, though always starting with 0.
 * @param matrix	The distance matrix.
 * @return			The optimal permutation.
 */
std::vector<size_t> optimal_permutation_fixed_start(const DistanceMatrix &matrix) {
	// Call out to our TSP solver:
	return tsp_open_end(
		[&matrix](size_t i) {
			assert(i + 1 < matrix.distances[0].size());
			return matrix.distances[0][i + 1];
		},
		[&matrix](size_t i, size_t j) {
			assert(i + 1< matrix.distances.size());
			assert(j + 1< matrix.distances.size());
			return matrix.distances[i + 1][j + 1];
		},
		matrix.distances.size() - 1
	);
}

auto visualize(SimpleVtkViewer &viewer,
               const ShellSpace &shell,
               const GeodesicPath &path,
               RGBColor color = {1.0, 0.0, 0.0}) {
	VtkPolyLineVisualization viz(color.r, color.g, color.b);

	std::vector<math::Vec3d> points;

	for (const auto &fl: path.path) {
		points.push_back(to_euclidean(shell, fl).surface_point);
	}
	viz.updateLine(points);

	viewer.addActor(viz.getActor());

	return viz;
}

REGISTER_VISUALIZATION(scan_fullpath) {
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Step 1: obtain a convex hull of the tree trunk:
	const auto &chull = compute_convex_hull_of_crown(tree_model);

	const auto aabb = mesh_aabb(tree_model.leaves_mesh);

	// Obtain the fruit positions:
	const auto &target_object_centers = computeFruitPositions(tree_model);

	// Project the fruit positions onto the convex hull:
	std::vector<ShellPoint> projected_fruit_positions;
	for (const auto &tgt: target_object_centers) {
		projected_fruit_positions.push_back(project_to_shell(chull, tgt));
	}

	std::vector<math::Vec3d> projected_fruit_positions_euclidean;
	for (const auto &fl: projected_fruit_positions) {
		projected_fruit_positions_euclidean.push_back(to_euclidean(chull, fl).surface_point);
	}

	// Create a vector of vec3d pairs:
	std::vector<math::Segment3d> projection_lines;
	for (size_t i = 0; i < target_object_centers.size(); ++i) {
		projection_lines.push_back(
			math::Segment3d{target_object_centers[i], projected_fruit_positions_euclidean[i]});
	}

	viewer.lockCameraUp();
	viewer.setCameraTransform(aabb.inflated(5.0).max(), aabb.center());

	viewer.run_puppeteer_thread([&](visualization::ThrottledRunQueue &rq) {
			rq.run_main_void([&](SimpleVtkViewer &v) {
				// Visualize the tree meshes
				visualize(v, tree_model);
			});
			rq.wait(50);

			rq.run_main_void([&](SimpleVtkViewer &v) {
				// Visualize the convex hull:
				visualize(v, chull);
			});
			rq.wait(50);

			rq.run_main_void([&](SimpleVtkViewer &v) {
				visualize(v, projection_lines, {1.0, 0.0, 1.0});
			});
			rq.wait(50);

			DistanceMatrixHook batch_hook = [&](const ShellPoint &target) {
				// auto geodesics = geodesics_one_to_many(chull, target, projected_fruit_positions);
				// std::vector<VtkPolyLineVisualization> path_visualizations;
				// rq.run_main_void([&](SimpleVtkViewer &v) {
				// 	for (const auto &geodesic: geodesics) {
				// 		auto viz = visualize(v, chull, geodesic);
				// 		path_visualizations.push_back(std::move(viz));
				// 	}
				// });
				// rq.wait(1);
				// rq.run_main_void([&](SimpleVtkViewer &v) {
				// 	for (auto &viz: path_visualizations) {
				// 		v.removeActor(viz.getActor());
				// 	}
				// 	path_visualizations.clear();
				// });
			};

			// TODO: this is missing the distances from the start point.
			const auto matrix = geodesic_distance_many_to_many(chull,
			                                                   projected_fruit_positions[0],
			                                                   projected_fruit_positions,
			                                                   batch_hook);

			auto optimal_permutation = optimal_permutation_fixed_start(matrix);

			GeodesicPath total_path;
			for (size_t i = 1; i < optimal_permutation.size(); ++i) {
				auto path = geodesic_one_to_one(chull,
				                                projected_fruit_positions[optimal_permutation[i]],
				                                projected_fruit_positions[optimal_permutation[i + 1]]);
				total_path.path.insert(total_path.path.end(), path.path.begin(), path.path.end());
			}

			// Visualize the total path:
			visualize(viewer, chull, total_path, {0.0, 1.0, 0.0});

			rq.wait(1000);
		}

	);
}
