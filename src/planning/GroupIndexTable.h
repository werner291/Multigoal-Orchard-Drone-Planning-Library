// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/19/24.
//

#ifndef GROUPINDEXTABLE_H
#define GROUPINDEXTABLE_H
#include <vector>
#include <cstddef>

namespace mgodpl {
	/**
	 * This struct provides a table of indices for goal samples, grouped by fruit.
	 * It also tracks the total number of samples.
	 *
	 * Specifically, it allows to look up two things:
	 * - Given a fruit index, the indices of the goal samples for that fruit.
	 * - Given a fruit index and a goal sample index, the goal sample index in the global list of goal samples.
	 *
	 * Note: these are NOT graph vertex IDs.
	 */
	class GroupIndexTable {
		size_t total_samples;
		std::vector<std::vector<size_t> > index_table;

	public:
		/**
		 * @brief Construct a group index table from a vector of counts of goal samples per fruit.
		 * @param counts	A vector of counts of goal samples per fruit.
		 */
		explicit GroupIndexTable(const std::vector<size_t> &counts);

		/**
		 * @brief Look up the indices of the goal samples for a given fruit.
		 * @param fruit_index	The index of the fruit.
		 * @return A vector of the indices of the goal samples for that fruit.
		 */
		[[nodiscard]] inline const std::vector<size_t> &for_fruit(size_t fruit_index) const {
			return index_table[fruit_index];
		}

		/**
		 * @brief Look up the global index of a goal sample.
		 * @param fruit_index	The index of the fruit.
		 * @param sample_index	The index of the goal sample within the fruit.
		 * @return		The global index of the goal sample.
		 */
		[[nodiscard]] inline const size_t &lookup(size_t fruit_index, size_t sample_index) const {
			return index_table[fruit_index][sample_index];
		}

		/**
		 * @brief Get the total number of goal samples.
		 */
		[[nodiscard]] inline size_t total() const {
			return total_samples;
		}
	};
} // mgodpl

#endif //GROUPINDEXTABLE_H
