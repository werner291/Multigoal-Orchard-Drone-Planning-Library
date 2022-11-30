// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_SURFLETVOLUME_H
#define NEW_PLANNERS_SURFLETVOLUME_H

#include "SegmentedPointCloud.h"
#include <CGAL/Orthogonal_incremental_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kernel/Type_equality_wrapper.h>
#include <CGAL/Simple_cartesian.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>
#include <Eigen/Core>

/**
 *
 * A model of a volume in 3D (though reasonably could be extended to any dimension) based on "surflets":
 * point-normal pairs that designate points on the boundary of the volume.
 *
 * Formally, a Surflet Volume is a set S of pairs (P,N) where P is a point in R^3, and N is a unit normal vector (in R^3).
 *
 * The volume is implicitly defined as the set of all points P' in R^3 where, given P as the closest surflet to P
 * in S, dot(P' - P, N) <= 0.
 *
 */
class SurfletVolume {

public:
	struct Surflet {
		Eigen::Vector3d point;
		Eigen::Vector3d normal;
	};

private:

	std::vector<Surflet> surflets; // TODO Will wanna make this a KD-tree or other spatial datastructure at some point

public:

	struct NearbySurflet {
		Surflet surflet;
		double distance_squared;
	};

	[[nodiscard]] NearbySurflet closest(const Eigen::Vector3d &query) const;

	SurfletVolume unionWith(const SurfletVolume &other);

	void add(Surflet s);

	[[nodiscard]] bool isInside(const Eigen::Vector3d &p) const;
};

#endif //NEW_PLANNERS_SURFLETVOLUME_H
