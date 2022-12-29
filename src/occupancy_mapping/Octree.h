// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_OCTREE_H
#define NEW_PLANNERS_OCTREE_H

#include <variant>
#include <memory>
#include <Eigen/Geometry>
#include "../utilities/math_utils.h"

template<typename SplitData, typename LeafData>
class Octree {

public:

	Eigen::AlignedBox3d box;

	struct LeafCell {
		LeafData data;
	};

	// Forward declaration of SplitCell since we use it in the Cell variant.
	struct SplitCell;

	// Cell, variant of either an internal cell (with children) or a leaf cell.
	using Cell = std::variant<SplitCell, LeafCell>;

	// A cell with 8 children. Children cover an octant of the parent,
	// in the order defined by OctantIterator in math_utils.h
	struct SplitCell {
		SplitData data;
		std::unique_ptr<std::array<Cell, 8> > children;

		/**
		 * @brief Check if all children of this SplitCell are LeafCells.
		 * @return `true` if all children are LeafCells, `false` otherwise.
		 */
		[[nodiscard]] bool all_children_are_leaves() const {
			return std::all_of(children->begin(),
							   children->end(),
							   [](const auto &child) { return std::holds_alternative<LeafCell>(child); });
		}

		/**
		 * @brief Check if all children of this SplitCell are LeafCells and if they all have the same data.
		 * @return `true` if all children are LeafCells with the same data, `false` otherwise.
		 */
		[[nodiscard]] bool has_uniform_leaves() const {
			if (!all_children_are_leaves()) {
				return false;
			}

			const LeafData &first_data = std::get<LeafCell>(children->operator[](0)).data;
			return std::all_of(children->begin() + 1,
							   children->end(),
							   [&](const auto &child) { return std::get<LeafCell>(child).data == first_data; });
		}

	};

	Cell root;

	explicit Octree(const Cell &root) : root(root) {
	}

	explicit Octree(const LeafData &root_data) : root(LeafCell{root_data}) {
	}

	static SplitCell split_by_copy(const LeafCell &cell, SplitData data) {
		SplitCell split_cell;
		split_cell.data = data;
		split_cell.children = std::make_unique<std::array<Cell, 8>>();
		for (size_t i = 0; i < 8; i++) {
			(*split_cell.children)[i] = LeafCell{cell.data};
		}
		return split_cell;
	}

	/**

		@brief Retrieve the leaf cell data at the given query point.

		@param query_point The point to retrieve data for.

		@return The data of the leaf cell at the given query point.

		@throws std::logic_error If the query point is not contained within the octree.
		*/
	LeafData get_leaf_data_at(const Eigen::Vector3d &query_point) const {
		// Ensure that the query point is contained within the octree's bounding box
		assert(box.contains(query_point));

		// Current cell that we are searching within
		const Cell *current_cell = &root;
		Eigen::AlignedBox3d current_box = box;

		// Iterate through the octree until we reach a leaf cell
		while (true) {

			if (auto *split_cell = std::get_if<SplitCell>(current_cell)) {
				// If the current cell is a split cell, find the child octant that contains the query point
				auto child_octant = math_utils::find_octant_containing_point(current_box, query_point);

				current_cell = &(*split_cell->children)[child_octant.i];
				current_box = child_octant.bounds;

			} else {
				// If the current cell is a leaf cell, return the data
				return std::get<LeafCell>(*current_cell).data;
			}
		}
	}

private:

	/**
	 * Internal: traverse the octree and call the given function on each leaf cell.
	 *
	 * @param box 					The bounding box of the current cell.
	 * @param cell 					The current cell.
	 * @param max_depth 			The maximum depth to traverse to, starting from the current cell.
	 * @param at_highest_lod 		Callback to call when the current cell is at the maximum depth.
	 * @param split_rule 			Callback to call whether a given leaf cell should be split.
	 */
	void traverse_from_cell(const Eigen::AlignedBox3d &box,
							Cell &cell,
							const int max_depth,
							const std::function<void(const Eigen::AlignedBox3d &, LeafCell &)> &at_highest_lod,
							const std::function<bool(const Eigen::AlignedBox3d &, LeafCell &)> &split_rule) {

		// If the cell is a leaf cell, check if it should be split.
		if (auto *leaf_cell = std::get_if<LeafCell>(&cell)) {
			if (max_depth == 0 || split_rule(box, *leaf_cell)) {
				// If the cell should be split, split it and recurse on the children
				cell = split_by_copy(*leaf_cell, SplitData{});
			} else {
				// If the cell should not be split, call the callback and return
				at_highest_lod(box, *leaf_cell);
				return;
			}
		}

		// If we got here, the cell is a split cell. Recurse on the children.
		SplitCell &split_cell = std::get<SplitCell>(cell);
		OctantIterator iter(box);
		// Iterate over the 8 child cells
		for (auto &child_cell: *split_cell.children) {
			// Recursively update the child cells with the current point cloud data
			traverse_from_cell(iter++.bounds, child_cell, max_depth - 1, at_highest_lod, split_rule);
		}

		// If all children are leaf cells with the same data, collapse the split cell into a leaf cell
		if (split_cell.has_uniform_leaves()) {
			cell = LeafCell{std::get<LeafCell>(split_cell.children->operator[](0)).data};
		}
	}

public:
	/**
	 * Traverse and update the octree from the root.
	 *
	 * @param max_depth 			The maximum depth of the traversal.
	 * @param at_highest_lod 		A function to call when the traversal reaches the highest LOD.
	 * @param split_rule 			A function to call to determine whether to split a cell, assuming that the cell is not at the maximum depth.
	 */
	void traverse(const int max_depth,
				  const std::function<void(const Eigen::AlignedBox3d &, LeafCell &)> &at_highest_lod,
				  const std::function<bool(const Eigen::AlignedBox3d &, LeafCell &)> &split_rule) {
		traverse_from_cell(box, root, max_depth, at_highest_lod, split_rule);
	}


};


#endif //NEW_PLANNERS_OCTREE_H
