// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_VECTOR_UTILS_H
#define NEW_PLANNERS_VECTOR_UTILS_H

#include <vector>
#include <cstddef>

/**
 * Reorders a vector by the given indexes.
 *
 * Note: Elements are moved from the original vector to the new vector,
 * so duplicate indexes are not allowed and may cause undefined behavior.
 *
 * @tparam T 				The type of the vector elements
 * @param vec 				The vector to reorder (pass by move)
 * @param ordering_indexes 	The indexes to reorder by (must be unique)
 * @return 					The reordered vector
 */
template<typename T>
std::vector<T> reorder_by_index(std::vector<T> vec, const std::vector<size_t> &ordering_indexes) {
	std::vector<T> result;

	for (auto i: ordering_indexes) {
		result.push_back(std::move(vec[i]));
	}

	return result;
}

#endif //NEW_PLANNERS_VECTOR_UTILS_H
