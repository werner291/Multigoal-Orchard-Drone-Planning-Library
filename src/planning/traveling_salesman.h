
#ifndef NEW_PLANNERS_TRAVELING_SALESMAN_H
#define NEW_PLANNERS_TRAVELING_SALESMAN_H

#include <functional>

/**
 * Determine an approximately optimal ordering of a given set of items/indices.
 *
 * Items are represented by indices only.
 *
 * The tour is assumed to start from some implicit item, visit all items in the set, and terminate at an arbitrary item in the set.
 *
 * @param from_start	A function that gives a distance between a given item index and some assumed start item.
 * @param between		A function that gives a distance between two given item indices.
 * @param n				The number of items in the set (all indices will be in the range 0..n-1).
 * @return				The tour, represented as a vector of indices.
 *
 * @throws PlanningTimeout if the timeout is exceeded.
 */
std::vector<size_t> tsp_open_end(
	const std::function<double(size_t)> &from_start,
	const std::function<double(size_t, size_t)> &between,
	size_t n);

/**
 * Determine an approximately optimal ordering of a given set of items/indices, where items may be grouped together.
 *
 * Items are identified as pairs of indices: a group index and an item index within that group.
 *
 * @param from_start 	A function that gives a distance between a given item and an implicit start item.
 * @param between 		A function that gives a distance between two given items.
 * @param sizes 		A vector of the number of items in each group.
 * @return 				The tour, represented as a vector of pairs of indices.
 *
 * @throws PlanningTimeout if the timeout is exceeded.
 */
std::vector<std::pair<size_t, size_t> >
tsp_open_end_grouped(const std::function<double(std::pair<size_t, size_t>)> &from_start,
                     const std::function<double(std::pair<size_t, size_t>, std::pair<size_t, size_t>)> &between,
                     const std::vector<size_t> &sizes);

#endif //NEW_PLANNERS_TRAVELING_SALESMAN_H
