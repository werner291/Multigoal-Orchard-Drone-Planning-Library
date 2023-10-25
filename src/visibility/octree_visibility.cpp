// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/23/23.
//

#include <chrono>
#include "octree_visibility.h"
#include "Octree.h"
#include "visibility_geometry.h"

namespace mgodpl::visibility {

	using namespace math;

	AABBd childAABB(const AABBd &parent, size_t child_index) {
		assert(child_index < 8);

		// Compute the child AABB.
		Vec3d center = parent.center();
		Vec3d half_size = parent.size() / 2.0;

		Vec3d min = {
				(child_index & 1) ? center.x() : center.x() - half_size.x(),
				(child_index & 2) ? center.y() : center.y() - half_size.y(),
				(child_index & 4) ? center.z() : center.z() - half_size.z()
		};

		return {
			min,
			min+half_size
		};

	}

	VisibilityOctree cast_occlusion_batch_sorting(const math::AABBd &base_volume,
												  const std::vector<math::Triangle> &triangles,
												  const math::Vec3d &eye) {

		VisibilityOctree occluded;


		// Sort by distance from the viewpoint, then process in order.
		for (const auto& [_distance, triangle] : sorted_by_distance(triangles, eye)) {
			cast_occlusion(base_volume, occluded, *triangle, eye, 5);
		}

		return occluded;

	}

	bool is_leaf_with_value(const VisibilityOctree::Node &node, bool value) {
		if (const auto& leaf = std::get_if<VisibilityOctree::LeafNode>(&node)) {
			return leaf->data == value;
		} else {
			return false;
		}
	}

	void try_merge(VisibilityOctree::Node &occluded) {// If the children are identical leaf nodes, then we can collapse them into a single leaf node.
		bool all_same = true;

		// All children must be leaf nodes.
		for (size_t i = 0; i < 8; ++i) {
			if (!std::holds_alternative<VisibilityOctree::LeafNode>(std::get<VisibilityOctree::SplitNode>(occluded).children->at(i))) {
				return;
			}
		}

		const auto& first_leaf = std::get<VisibilityOctree::SplitNode>(occluded).children->at(0);

		for (size_t i = 1; i < 8; ++i) {
			if (std::get<VisibilityOctree::LeafNode>(first_leaf).data != std::get<VisibilityOctree::LeafNode>(std::get<VisibilityOctree::SplitNode>(occluded).children->at(i)).data) {
				all_same = false;
				break;
			}
		}

		if (all_same) {
			occluded = VisibilityOctree::LeafNode { std::get<VisibilityOctree::LeafNode>(first_leaf).data };
		}
	}

	void cast_occlusion_internal(
			const math::AABBd &volume,
			VisibilityOctree::Node &occluded,
			const std::array<Ray,3> &rays,
			const size_t max_depth) {

		// If it's already a leaf that is occluded, skip it.
		if (is_leaf_with_value(occluded, true)) {
			return;
		}

		// Grab the affected volume.
		const auto& intersection_aabb = aabbInAABB(volume, rays);

		// Three cases: no intersection, partial intersection, contained.

		// No intersection: nothing to do.
		if (!intersection_aabb) {
			return;
		}

		// If there's a full intersection, then the entire volume is occluded.
		// At max depth, a partial intersection is treated as a full intersection.
		if (intersection_aabb->volume() - 1e-10 >= volume.volume() || max_depth == 0) {
			occluded = VisibilityOctree::LeafNode { true };
			return;
		}

		// There's a partial intersection; split the node if necessary.
		if (const auto& leaf = std::get_if<VisibilityOctree::LeafNode>(&occluded)) {

			occluded = VisibilityOctree::SplitNode {
				// Can we use make_unique somehow?
				.children = std::unique_ptr<std::array<VisibilityOctree::Node,8>>(new std::array<VisibilityOctree::Node,8> {VisibilityOctree::LeafNode{leaf->data}}) // NOLINT(*-make-unique)
			};

		}

		// Recurse on the children.
		for (size_t i = 0; i < 8; ++i) {
			cast_occlusion_internal(childAABB(volume, i), std::get<VisibilityOctree::SplitNode>(occluded).children->at(i), rays, max_depth - 1);
		}

		try_merge(occluded);

	}

	void cast_occlusion(const math::AABBd &base_volume,
						VisibilityOctree &occluded,
						const math::Triangle &triangle,
						const math::Vec3d &eye,
						int depth) {

		// The occluded volume is the convex hull of the three rays, limited to the parent AABB.
		// Rays are offset a bit to account for the idea of "boundary cells".
		const std::array<Ray, 3> rays{occluded_ray(eye, triangle.a, 0.0),
									  occluded_ray(eye, triangle.b, 0.0),
									  occluded_ray(eye, triangle.c, 0.0)};

		cast_occlusion_internal(base_volume, occluded._root, rays,
								depth);

	}

	struct RaySlice {
		const Ray ray;
		RangeInclusiveD t_range;
	};

	VisibilityOctree::Node cast_occlusion_batch_internal_dnc(
			const math::AABBd &volume,
			const std::vector<std::array<RaySlice,3>>& rays,
			const math::Vec3d &eye
			) {

		double x_min = volume.min().x();
		double x_max = volume.max().x();
		double x_split = (x_min + x_max) / 2.0;

		std::vector<std::array<RaySlice,3>> rays_sub;

		// Expanding AABB idea?! An expanding cube around the eye point, and maintain the density of stuff on the surface?

		for (const auto& ray_slice : rays) {
			for (const auto& ray : ray_slice) {
				double ox = ray.ray.origin().x();
				double dx = ray.ray.direction().x();
				double crossover_t = (x_split - ox) / dx;

				if (ray.t_range.contains(crossover_t)) {
					rays_sub.push_back(ray_slice);
					break;
				}
			}


		}

	};

	VisibilityOctree
	cast_occlusion_batch_dnc(const math::AABBd &base_volume, std::vector<math::Triangle> triangles, const math::Vec3d &eye) {

//		return VisibilityOctree { cast_occlusion_batch_internal_dnc(base_volume, triangles, eye) };

	}
}