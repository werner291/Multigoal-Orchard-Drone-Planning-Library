// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#ifndef NEW_PLANNERS_SIGNED_DISTANCE_FIELDS_H
#define NEW_PLANNERS_SIGNED_DISTANCE_FIELDS_H

#include <utility>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include "math_utils.h"
#include "EigenExt.h"

/**
 * A module for various utilities related to signed distance fields with gradients.
 *
 * By convention, distance values are positive outside of the object and negative inside; normals
 * are simply the gradient (gradient is always unit-length since the distance is a Euclidean distance).
 *
 * Static dispatch is used to avoid virtual function calls.
 */
namespace sdf {

	using namespace EigenExt;

	/**
	 * A signed distance field value, consisting of a distance and a normal.
	 */
	struct SdfValue {
		/// Signed distance, positive outside of the object and negative inside.
		double signed_distance;
		/// Normal, always unit-length, pointing out of the object.
		UVector3d normal;

		/**
		 * Reconstruct the position of the point on the surface of the object closest to the query point.
		 *
		 * @param p 		The query point originally passed to the SDF.
		 * @return 			The position of the point on the surface of the object closest to the query point.
		 */
		[[nodiscard]] Eigen::Vector3d reconstruct_closest_point(const Eigen::Vector3d &p) const {
			return p - signed_distance * normal.getVector();
		}
	};

	/**
	 * A signed distance field to a zero-volume ray.
	 *
	 * Signed distances will always be positive outside of the ray and zero on the ray. There is no inside.
	 *
	 * Gradients point out from the ray, which means away from the origin of the ray when beyond the end,
	 * otherwise we look for the closest point on the ray to the query point.
	 */
	struct RaySdf {
		/// The underlying ray.
		Ray3d ray;

		/// Evaluate the Sdf.
		SdfValue operator()(const Eigen::Vector3d &p) const;
	};

	struct PlaneSdf {
		/// The underlying plane.
		Plane3d plane;

		/// Evaluate the Sdf.
		SdfValue operator()(const Eigen::Vector3d &p) const;
	};

	/*
	 * A union of two SDFs. Returns the minimum of the two SDFs.
	 *
	 * The interior volume of the union is the union of the interior volumes of the two SDFs;
	 * conversely, the exterior volume of the union is the intersection of the exterior volumes
	 * of the two SDFs.
	 *
	 * As a shorthand, one can simply use the + operator to create a union of two SDFs.
	 */
	template<typename L, typename R>
	struct UnionSdf {
		/// The left SDF.
		L left;
		/// The right SDF.
		R right;

		/// Evaluate the Sdf.
		SdfValue operator()(const Eigen::Vector3d &p) const {

			// We evaluate both
			SdfValue left_value = left(p);
			SdfValue right_value = right(p);

			// And return the minimum
			if (left_value.signed_distance < right_value.signed_distance) {
				return left_value;
			} else {
				return right_value;
			}
		}
	};

	template<typename Sdf>
	struct InvertedSdf {
		Sdf sdf;

		SdfValue operator()(const Eigen::Vector3d &p) const {
			SdfValue value = sdf(p);
			value.signed_distance = -value.signed_distance;
			value.normal = -value.normal;
			return value;
		}
	};

	/**
	 * A signed SDF of a triangle with an open side.
	 *
	 * There are two cases:
	 * - If the perpendicular projection of the query point onto the plane of the triangle is inside the triangle,
	 *   treat the triangle as simply a plane aligned with the triangle.
	 *
	 * - Otherwise, return the minimum signed distance to the two rays that form the edges of the triangle.
	 *   The sign of the signed distance is determined by corners_inside.
	 *
	 * A cross-section of the volume looks something like this:
	 *
	 * 	                  |			Outside			 |
	 * 	                  |							 |
	 * 	corners_inside?   +---- Triangle surface ----+   corners_inside?
	 * 	                  |                          |
	 * 	                  |			Inside			 |
	 *
	 */
	struct OpenTriangleSdf {
		OpenTriangle triangle;
		bool corners_outside = false;

		OpenTriangleSdf(const Eigen::Vector3d &p1, const std::array<EigenExt::UVector3d, 2> &rays, bool cornersOutside)
				: triangle{p1, *rays[0], *rays[1]}, corners_outside(cornersOutside) {
		}

		SdfValue operator()(const Eigen::Vector3d &p) const {

			// Compute the barycentric coordinates of the projection of p onto the plane of the triangle
			Eigen::Vector3d barycentric = project_barycentric(p,
															  triangle.apex,
															  triangle.apex + triangle.dir1,
															  triangle.apex + triangle.dir2);

			// If it lies in the open triangle, treat it as a plane and return the signed distance to the plane
			if (barycentric[0] <= 1 && 0 <= barycentric[1] && 0 <= barycentric[2]) {
				return {(p - triangle.apex).dot(*triangle.normal()), triangle.normal()};
			} else {
				// Otherwise, return the minimum signed distance to the two rays that form the edges of the triangle
				// note that this implicitly takes care of the case where the point lies beyond the apex of the triangle

				SdfValue sf = UnionSdf<RaySdf, RaySdf>{RaySdf{Ray3d{triangle.apex, triangle.dir1}},
													   RaySdf{Ray3d{triangle.apex, triangle.dir2}}}(p);

				//				if (corners_outside) {
				//					sf.signed_distance = -sf.signed_distance;
				//					sf.normal = -sf.normal;
				//				}

				return sf;
			}
		}

	};

	SdfValue evalSDF(Eigen::Isometry3d &eePose, const Eigen::Vector3d &p);

	template<typename SdfA, typename SdfB>
	UnionSdf<SdfA, SdfB> operator+(SdfA a, SdfB b) {
		return UnionSdf<SdfA, SdfB>{a, b};
	}

	/*
	 * Transform an sdf by an Isometry3d.
	 */
	template<typename Sdf>
	class IsometrySdf {

		// The underlying sdf
		Sdf sdf;
		// The transformation (note: INVERSE of the one given to the constructor)
		Eigen::Isometry3d point_transform;
		Eigen::Quaterniond normal_transform;

	public:
		/**
		 * Construct an Isometric transformation of an SDF.
		 * @param sdf 				The original SDF.
		 * @param isometry 			The transformation to apply to the SDF; will be inverted such that it is applied to the points instead.
		 */
		IsometrySdf(const Eigen::Isometry3d &isometry, Sdf sdf)
				: sdf(sdf), point_transform(isometry.inverse()), normal_transform(isometry.rotation()) {
		}

		SdfValue operator()(const Eigen::Vector3d &p) const {
			SdfValue sd = sdf(point_transform * p);
			return {sd.signed_distance, normal_transform * sd.normal};
		}
	};

	using ViewPyramidSdf = UnionSdf<UnionSdf<UnionSdf<PlaneSdf, PlaneSdf>, UnionSdf<PlaneSdf, PlaneSdf> >, PlaneSdf>;

	ViewPyramidSdf open_view_pyramid_local(const Eigen::Vector2d &plane_slopes);

}

#endif //NEW_PLANNERS_SIGNED_DISTANCE_FIELDS_H
