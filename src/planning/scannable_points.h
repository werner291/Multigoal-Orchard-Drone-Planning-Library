// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_SCANNABLE_POINTS_H
#define MGODPL_SCANNABLE_POINTS_H

#include <memory>
#include <vector>
#include "../math/Vec3.h"
#include "../math/AABB.h"

namespace mgodpl {
	/**
	 * @brief A struct representing a point on a surface.
	 *
	 * This struct encapsulates the position and normal vector of a point on a surface.
	 */
	struct SurfacePoint {
		math::Vec3d position; ///< The position of the point on the surface.
		math::Vec3d normal; ///< The normal vector at the point on the surface.
	};

	/**
	 * @brief A struct encapsulating scannable points parameters.
	 *
	 * This struct encapsulates the maximum distance, minimum distance, and maximum angle for scanning checks,
	 * as well as a vector of SurfacePoint objects for which scanning is to be performed.
	 */
	struct ScannablePoints {
		double max_distance; ///< The maximum distance for scanning checks.
		double min_distance; ///< The minimum distance for scanning checks.
		double max_angle; ///< The maximum angle for scanning checks.
//		std::optional <std::shared_ptr<MeshOcclusionModel>> occlusion_model; ///< The occlusion mesh to use for visibility checks.
		std::vector<SurfacePoint> surface_points; ///< The vector of SurfacePoint objects for which scanning is to be performed.

		using PointId = size_t; ///< An identifier for a point in ScannablePoints.
	};

	/**
	 * @brief Computes the Axis-Aligned Bounding Box (AABB) for a given cluster of scannable points.
	 *
	 * The AABB is inflated to include the maximum scan distance in all directions.
	 *
	 * @param cluster A ScannablePoints object representing a cluster of points.
	 * @return The computed AABB for the given cluster.
	 */
	math::AABBd computeAABBForCluster(const ScannablePoints &cluster);

	/**
	 * @brief Computes the Axis-Aligned Bounding Boxes (AABBs) for a vector of clusters of scannable points.
	 *
	 * The AABBs are inflated to include the maximum scan distance in all directions.
	 *
	 * @param clusters A vector of ScannablePoints objects, each representing a cluster of points.
	 * @return A vector of computed AABBs for the given clusters.
	 */
	std::vector<math::AABBd> computeAABBsForClusters(const std::vector<ScannablePoints> &clusters);
}

#endif //MGODPL_SCANNABLE_POINTS_H
