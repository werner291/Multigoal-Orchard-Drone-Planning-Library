// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"

HierarchicalBoundaryCellAnnotatedRegionOctree::HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
																							 const double baseEdgeLength)
		: center(center), base_edge_length(baseEdgeLength) {
}
