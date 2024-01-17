// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/3/24.
//

#ifndef MGODPL_QUICKPROBE_H
#define MGODPL_QUICKPROBE_H

#include "longitude_sweep.h"
//
//namespace mgodpl {
//
//
//   /**
//	* This designates a spherical parallelogram on the unit sphere,
//	* were the bases are aligned with the sphere's meridians,
//	* and the sides are arbitrary great circles.
//	*
//	* The interior thus forms a continuous set of points on the sphere.
// 	*/
//	struct VerticalSpherePatch {
//
//		/// The longitude range of the patch, designating the longitudes of the two meridians.
//		LongitudeRange longitude_range;
//
//		/// The latitude ranges of the patch, designating the latitudes of the two parallelogram sides.
//		/// One of these may be a singleton range, in which case the patch is a spherical triangle.
//		LatitudeRange left_edge, right_edge;
//
//	};
//
//	/**
//	 * @brief 	Create the vertical sphere patches that represent the latitude/longitude ranges
//	 *          that cannot be used as safe probing angles due to this triangle.
//	 *
//	 * @param triangle 			The triangle that forms an obstacle.
//	 * @param center 			The point that the probe is pointing at.
//	 * @param arm_radius 		The radius of the arm; the larger, the more clearance is needed.
//	 * @param probe_margin 		How far from the `center` the tip of the arm is.
//	 * @return 					The patches of latitude/longitude pairs that are not safe for probing.
//	 */
//	std::array<VerticalSpherePatch, 2> occupied_by_triangle(const Triangle &triangle,
//															 const math::Vec3d &center,
//															 double arm_radius,
//															 double probe_margin);
//
//	/**
//	 * @brief A class wrapper around the QuickProbe algorithm; it effectively provides an
//	 * interface over all the free VerticalSpherePatches that form the safe sampling space
//	 * for probe angles.
//	 */
//	class QuickProbe {
//
//		/// The patches of latitude/longitude pairs that are not safe for probing;
//		/// We wish to effectively find the complement of the union of these.
//		const std::vector<VerticalSpherePatch> occupied_patches;
//
//		/**
//		 * @brief Represents a rectangular latitude/longitude patch on the sphere,
//		 * the occupied patches that intersect with it, and a pivot point that
//		 * splits it into four sub-patches.
//		 */
//		struct StackFrame {
//			/// An iterator to the end of the range of occupied patches that are within the current working patch.
//			std::vector<VerticalSpherePatch>::iterator occupied_in_patch_end;
//
//			/// The pivot point that splits the current working patch into four sub-patches.
//			RelativeVertex pivot;
//
//			/// The latitude and longitude ranges that form the current working patch.
//			LongitudeRange longitude_range;
//			LatitudeRange left_edge, right_edge;
//		};
//
//		/// The stack of working patches.
//		std::vector<StackFrame> stack;
//
//	public:
//
//		/**
//		 * Constructor.
//		 *
//		 * Constructs a QuickProbe object for the given set of triangles, centered around the given point;
//		 * the algorithm will yield patches of latitude/longitude ranges that are to position the arm at.
//		 *
//		 * @param triangles 	   	The triangles that form obstacles.
//		 * @param center 			The point that the probe is pointing at.
//		 * @param arm_radius 		The radius of the arm; the larger, the more clearance is needed.
//		 */
//		QuickProbe(const std::vector<Triangle> &triangles,
//				   const math::Vec3d& center,
//				   double arm_radius);
//
//		/**
//		 * @bried 	Check whether a new patch can be found. If not, the algorithm is done..
//		 * @return 	True if there is a next patch, false otherwise.
//		 */
//		bool has_next() const;
//
//		/**
//		 * @brief 	Get the next patch.
//		 * @return 	The next patch of safe probing angles.
//		 */
//		VerticalSpherePatch next();
//
//	};
//
//}

#endif //MGODPL_QUICKPROBE_H
