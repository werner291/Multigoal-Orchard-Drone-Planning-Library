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

};


#endif //NEW_PLANNERS_OCTREE_H
