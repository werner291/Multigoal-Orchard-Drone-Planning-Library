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
#include <iostream>
#include <Eigen/Geometry>
#include "../utilities/math_utils.h"

/**
 * A small struct counting the number of nodes in a tree.
 *
 * Distinguishes between internal nodes and leaf nodes.
 */
struct NodeCount {
	size_t leaf_count = 0; /// The number of leaf nodes in the tree.
	size_t split_count = 0; /// The number of split nodes in the tree.
};

/**
 * A class template to represent an octree that can be annotated with data at every node;
 * a distinction is made between internal nodes and leaf nodes.
 *
 * NOte that nodes do not store their own bounding box; this is derived on-the-fly
 * during tree traversal, which avoid storing this data but makes traversals that
 * start from the middle of the tree more expensive.
 *
 * @tparam SplitData 		The type of data to be stored at internal nodes.
 * @tparam LeafData 		The type of data to be stored at leaf nodes.
 */
template<typename SplitData, typename LeafData>
class Octree {

public:

	/// The root node's bounding box.
	Eigen::AlignedBox3d box;

	/// A leaf cell.
	struct LeafCell {
		LeafData data;
	};

	/// Forward declaration of SplitCell since we use it in the Cell variant.
	struct SplitCell;

	/**
	 * A Cell; either a leaf cell or a split cell.
	 */
	struct Cell {

		/// Underlying variant type; either a leaf cell or a split cell.
		std::variant<SplitCell, LeafCell> cell;

		explicit Cell(std::variant<SplitCell, LeafCell> cell) : cell(std::move(cell)) {
		}

		explicit Cell(SplitData split_data) : cell(SplitCell(std::move(split_data))) {
		}

		explicit Cell(LeafData leaf_data) : cell(LeafCell(std::move(leaf_data))) {
		}

		explicit Cell(SplitCell split_cell) : cell(std::move(split_cell)) {
		}

		explicit Cell(LeafCell leaf_cell) : cell(std::move(leaf_cell)) {
		}

		/**
		 * Check if this cell is a split cell.
		 * @return True if this cell is a split cell, false otherwise.
		 */
		[[nodiscard]] bool is_split() const {
			return std::holds_alternative<SplitCell>(cell);
		}

		/**
		 * Check if this cell is a leaf cell.
		 * @return True if this cell is a leaf cell, false otherwise.
		 */
		[[nodiscard]] bool is_leaf() const {
			return std::holds_alternative<LeafCell>(cell);
		}

		/**
		 * Get the SplitCell of this cell.
		 */
		[[nodiscard]] SplitCell &get_split() {
			return std::get<SplitCell>(cell);
		}

		/**
		 * Get the LeafCell of this cell.
		 */
		[[nodiscard]] LeafCell &get_leaf() {
			return std::get<LeafCell>(cell);
		}

		/**
		 * Check whether this is a split cell and all of its children are leaf cells.
		 * @return True if this is a split cell and all of its children are leaf cells, false otherwise.
		 */
		[[nodiscard]] bool is_split_with_all_leaves() const {
			if (is_split()) {
				SplitCell split_cell = std::get<SplitCell>(cell);
				return std::all_of(split_cell.children.begin(), split_cell.children.end(), [](const Cell &child_cell) {
					return child_cell.is_leaf();
				});
			}
			return false;
		}

		/**
		 * Split a leaf cell, creating 8 children and copying the data from the leaf cell into the children.
		 * @param data 		The data to place into the split cell.
		 *
		 * Asserts that this cell is a leaf cell.
		 */
		void split_by_copy(SplitData data);

		/**
		 * Merge a split cell, creating a leaf cell. Cell data will be a move of the data from the first child.
		 * @return 	The data held by the split cell, by move.
		 */
		SplitData merge_cell() {
			auto split = std::get<SplitCell>(cell);
			auto first_leaf = std::get<LeafCell>(split.children[0].cell);

			cell = LeafCell{.data = first_leaf.data};

			return std::move(split.data);
		}
	};

	/// A cell with 8 children. Children cover an octant of the parent,
	/// in the order defined by OctantIterator in math_utils.h
	struct SplitCell {

		/// User data stored at this node.
		SplitData data;

		/// The children of this node. Never null; leaf nodes use the LeafNode struct instead.
		std::unique_ptr<std::array<Cell, 8> > children;

		/**
		 * @brief Check if all children of this SplitCell are LeafCells.
		 *
		 * @return `true` if all children are LeafCells, `false` otherwise.
		 */
		[[nodiscard]] bool all_children_are_leaves() const {
			// Check if all children are leaf nodes.
			return std::all_of(children->begin(),
							   children->end(),
							   [](const auto &child) { return std::holds_alternative<LeafCell>(child); });
		}

		/**
		 * @brief Check if all children of this SplitCell are LeafCells and if they all have the same data.
		 *
		 * @note Requires that operator== is defined for LeafData.
		 *
		 * @return `true` if all children are LeafCells with the same data, `false` otherwise.
		 */
		[[nodiscard]] bool has_uniform_leaves() const {

			// Check if all children are leaf nodes; if not, return false.
			if (!all_children_are_leaves()) {
				return false;
			}

			// Check if all children have the same data as the first child.
			const LeafData &first_data = std::get<LeafCell>(children->operator[](0)).data;
			return std::all_of(children->begin() + 1,
							   children->end(),
							   [&](const auto &child) { return std::get<LeafCell>(child).data == first_data; });
		}

	};

	/// The root node of the tree; may be a leaf node or a split node.
	Cell root;

	/**
	 * @brief Construct a new Octree object from a cell with possible subtree.
	 */
	explicit Octree(Cell root) : root(std::move(root)) {
	}

	/**
	 * @brief Construct a new Octree as a single leaf with the given data.
	 * @param root_data 		The data to store at the root node.
	 */
	explicit Octree(LeafData root_data) : root(std::move(LeafCell{root_data})) {
	}

	/**
	 * @brief Construct a SplitCell from a LeafCell, storing the given SplitData at the new node,
	 * and copying the currently-held LeafData to all 8 children.
	 *
	 * @param cell		The LeafCell to split.
	 * @param data		The SplitData to store at the new node.
	 * @return			The new SplitCell.
	 */
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
	 * @brief Retrieve the leaf cell data at the given query point.
	 *
	 * @param query_point The point to retrieve data for.
	 *
	 * @return The data of the leaf cell at the given query point.
	 *
	 * Causes assertion failure if the query point is not inside the tree's bounding box.
	 */
	LeafData get_leaf_data_at(const Eigen::Vector3d &query_point) const {

		// Ensure that the query point is contained within the octree's bounding box
		assert(box.contains(query_point));

		// Current cell that we are searching within
		const Cell *current_cell = &root;
		Eigen::AlignedBox3d current_box = box;

		// Iterate through the octree until we reach a leaf cell
		while (auto *split_cell = std::get_if<SplitCell>(current_cell)) {

			// If the current cell is a split cell, find the child octant that contains the query point
			auto child_octant = math_utils::find_octant_containing_point(current_box, query_point);

			// Update the current cell and bounding box to the child cell
			current_cell = &(*split_cell->children)[child_octant.i];
			current_box = *child_octant;
		}

		// If the current cell is a leaf cell, return the data
		return std::get<LeafCell>(*current_cell).data;
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
			if (max_depth != 0 && split_rule(box, *leaf_cell)) {
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
			traverse_from_cell(*iter++, child_cell, max_depth - 1, at_highest_lod, split_rule);
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


	/**
	 * Count the number of leaf and split cells in the octree.
	 *
	 * @return A struct containing the number of leaf and split cells.
	 */
	[[nodiscard]] NodeCount count_nodes() const {

		// Keep track of the number of leaf and split cells, starting at zero.
		NodeCount count {
			.leaf_count = 0,
			.split_count = 0
		};

		// A queue of the cells to visit after this one; initialize with the root cell.
		std::vector<const Cell *> cell_queue = {&root};

		// Keep visiting cells until the queue is empty.
		while (!cell_queue.empty()) {

			// Get the next cell to visit.
			const Cell *cell = cell_queue.back();
			cell_queue.pop_back();

			// If the cell is a leaf cell, increment the leaf cell count.
			if (std::holds_alternative<LeafCell>(*cell)) {
				count.leaf_count++;
			} else {
				// If the cell is a split cell, increment the split cell count...
				count.split_count++;

				// ...and add the children to the queue.
				const SplitCell &split_cell = std::get<SplitCell>(*cell);
				for (const Cell &child_cell: *split_cell.children) {
					cell_queue.push_back(&child_cell);
				}
			}
		}

		return count;
	}


};

template<typename SplitData, typename LeafData>
void Octree<SplitData, LeafData>::Cell::split_by_copy(SplitData data) {

	const LeafData &leaf_data = std::get<LeafCell>(this->cell).data;

	std::array<Cell, 8> children{Cell{LeafCell{leaf_data}}, Cell{LeafCell{leaf_data}}, Cell{LeafCell{leaf_data}},
								 Cell{LeafCell{leaf_data}}, Cell{LeafCell{leaf_data}}, Cell{LeafCell{leaf_data}},
								 Cell{LeafCell{leaf_data}}, Cell{LeafCell{leaf_data}}};

	SplitCell split_cell;
	split_cell.data = std::move(data);
	split_cell.children = std::make_unique<std::array<Cell, 8>>(std::move(children));

	cell = {std::move(split_cell)};
}

#endif //NEW_PLANNERS_OCTREE_H
