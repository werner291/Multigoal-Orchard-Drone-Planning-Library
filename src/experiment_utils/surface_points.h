// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-1-24.
//

#ifndef MGODPL_SURFACE_POINTS_H
#define MGODPL_SURFACE_POINTS_H

#include "../math/Vec3.h"
#include "../planning/MeshOcclusionModel.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/scannable_points.h"
#include "TreeMeshes.h"

namespace mgodpl {

	/**
	 * Generate a random barycentric coordinate uniformly distributed over the triangle.
	 *
	 * @param rng	The random number generator.
	 * @return		The random barycentric coordinate.
	 */
	math::Vec3d random_barycentric(random_numbers::RandomNumberGenerator &rng);

	/**
	 * Compute the cumulative areas of the triangles in a mesh, for uniform sampling.
	 * @param mesh 			The mesh to compute the cumulative areas for.
	 * @return 				A vector of cumulative areas, of the same size as the number of triangles in the mesh.
	 */
	std::vector<double> triangle_cumulative_areas(const Mesh &mesh);

	/**
	 * @brief Uniformly sample a point on a mesh.
	 *
	 * @param rng 					Random number generator
	 * @param mesh 					Mesh to sample from
	 * @param cumulative_areas 		Cumulative areas of the triangles in the mesh (see triangle_cumulative_areas)
	 * @return 						A point on the mesh including position and normal
	 */
	SurfacePoint sample_point_on_mesh(random_numbers::RandomNumberGenerator &rng,
									  const Mesh &mesh,
									  const std::vector<double> &cumulative_areas);

	/**
	 * This function samples points on a mesh surface. It first calculates the cumulative areas of all triangles in the mesh.
	 * Then, it generates random points on the mesh surface by selecting a triangle based on its area and generating a random
	 * point within it.
	 *
	 * @param rng 			A reference to a random number generator.
	 * @param mesh 			A reference to the mesh from which points are to be sampled.
	 * @param num_points 	The number of points to be sampled from the mesh.
	 *
	 * @return A vector of SurfacePoint structures. Each SurfacePoint contains a position and a normal vector.
	 */
	std::vector<SurfacePoint> sample_points_on_mesh(random_numbers::RandomNumberGenerator &rng,
													const Mesh &mesh,
													size_t num_points);

	/**
	 * @brief Creates a ScannablePoints object.
	 *
	 * This function encapsulates the `sample_points_on_mesh` function and returns a `ScannablePoints` object.
	 * The `ScannablePoints` object contains the maximum distance, minimum distance, and maximum angle for scanning checks,
	 * as well as a vector of SurfacePoint objects for which scanning is to be performed.
	 *
	 * @param rng A reference to a random number generator.
	 * @param mesh A reference to the mesh from which points are to be sampled.
	 * @param num_points The number of points to be sampled from the mesh.
	 * @param max_distance The maximum distance for scanning checks.
	 * @param min_distance The minimum distance for scanning checks.
	 * @param max_angle The maximum angle for scanning checks.
	 * @param occlusion_model The occlusion model to use for visibility checks.
	 * @return A ScannablePoints object.
	 */
	ScannablePoints createScannablePoints(random_numbers::RandomNumberGenerator &rng,
										  const Mesh &mesh,
										  size_t num_points,
										  double max_distance,
										  double min_distance,
										  double max_angle,
										  std::optional<std::shared_ptr<MeshOcclusionModel>> occlusion_model = std::nullopt);

	/**
	 * @brief Creates a vector of ScannablePoints for all fruit meshes in the tree model.
	 *
	 * This function iterates over all fruit meshes in the tree model and creates ScannablePoints
	 * for each mesh using the provided parameters.
	 *
	 * @param tree_model The tree model containing fruit meshes.
	 * @param rng The random number generator.
	 * @param num_points The number of points to sample on each mesh. (Not in total!)
	 * @param max_distance The maximum distance for scanning checks.
	 * @param min_distance The minimum distance for scanning checks.
	 * @param max_angle The maximum angle for scanning checks.
	 * @return A vector of ScannablePoints for all fruit meshes in the tree model.
	 */
	std::vector<ScannablePoints> createAllScannablePoints(
			const tree_meshes::TreeMeshes &tree_model,
			random_numbers::RandomNumberGenerator &rng,
			size_t num_points,
			double max_distance,
			double min_distance,
			double max_angle);

	/**
	 * @brief Checks if a point is visible from a given position.
	 *
	 * This function takes a ScannablePoints object, a point index, and a Vec3d object representing the eye position.
	 * It computes the visibility of the point from the eye position based on the distance and angle.
	 * A point is considered visible if it is within the maximum distance and the angle between the point's normal
	 * and the vector from the point to the eye is less than the maximum angle.
	 *
	 * Note: This function does not account for occlusions. It only checks the visibility based on distance and angle.
	 *
	 * @param scannable_points A ScannablePoints object. Each SurfacePoint object in ScannablePoints represents a point in 3D space
	 *                         and has a position and a normal. The ScannablePoints object also contains the maximum distance and
	 *                         maximum angle to consider a point visible.
	 * @param point_index      The index of the point in the ScannablePoints object to check for visibility.
	 * @param eye_position     A Vec3d object representing the position of the eye in 3D space.
	 * @return                 A boolean value. If true, the point is visible from the eye position. If false, the point is not visible.
	 */
	bool is_visible(const ScannablePoints &scannable_points,
					ScannablePoints::PointId point_index,
					const math::Vec3d &eye_position);

	/**
	 * @brief Checks if a point is visible from a given position and direction.
	 *
	 * This function checks if a point is visible from a given position and direction,
	 * considering a maximum and minimum distance, a maximum scan angle, a field of view angle,
	 * and a mesh occlusion model.
	 *
	 * @param point The point to check visibility for.
	 * @param eye_pos The position from which to check visibility.
	 * @param eye_forward The eye direction to use for field-of-view checks.
	 * @param max_distance The maximum distance for the point to be considered visible.
	 * @param min_distance The minimum distance for the point to be considered visible.
	 * @param max_scan_angle The maximum angle between the point's normal and the vector from the point to the position for the point to be considered visible.
	 * @param fov_angle The field of view angle.
	 * @param mesh_occlusion_model The mesh occlusion model to use for visibility checks.
	 * @return true if the point is visible, false otherwise.
	 */
	bool is_visible(const SurfacePoint &point,
					const math::Vec3d &eye_pos,
					const math::Vec3d &eye_forward,
					double max_distance,
					double min_distance,
					double max_scan_angle,
					double fov_angle,
					const MeshOcclusionModel &mesh_occlusion_model);

	/**
	 * @brief Updates the visibility status of a set of points from a given eye position.
	 *
	 * This function takes a ScannablePoints object, a Vec3d object representing the eye position,
	 * and a SeenPoints object representing whether each point has ever been seen.
	 * It updates the visibility status of each point in the SeenPoints object. A point is considered visible
	 * if it is within the maximum distance and the angle between the point's normal
	 * and the vector from the point to the eye is less than the maximum angle.
	 *
	 * @param scannable_points A ScannablePoints object. Each SurfacePoint object in ScannablePoints represents a point in 3D space
	 *                         and has a position and a normal. The ScannablePoints object also contains the maximum distance and
	 *                         maximum angle to consider a point visible.
	 * @param eye_position     A Vec3d object representing the position of the eye in 3D space.
	 * @param seen_points      A SeenPoints object. Each value in the SeenPoints object corresponds to a point in the ScannablePoints object.
	 *                         If the value is true, the point has ever been seen from the eye position. If the value is false, the point has never been seen.
	 */
	size_t update_visibility(const ScannablePoints &scannable_points,
							 const math::Vec3d &eye_position,
							 SeenPoints &seen_points);
}

#endif //MGODPL_SURFACE_POINTS_H
