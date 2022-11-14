
#ifndef NEW_PLANNERS_ONLINEORDEROPTIMIZATION_H
#define NEW_PLANNERS_ONLINEORDEROPTIMIZATION_H

#include <functional>
#include <vector>

template<typename V>
class OnlineOrderOptimization {

protected:
	/// A distance function for consecutive elements in the order.
	std::function<double(const V &, const V &)> distance_function;
	/// A distance function for the first element in the order.
	std::function<double(const V &)> first_distance_function;

	/// A callback function when a new better order was discovered.
	std::function<void(const std::vector<V> &)> new_best_order_callback;

public:

	/**
	 * Create a new AnytimeOptimalInsertion with the given distance functions.
	 *
	 * Note that these functions do not need to be constant and may refer to external variables.
	 * (TODO: Might be better to have some kind of notification mechanism instead... we'll see.)
	 *
	 * @param firstDistanceFunction 		A distance function for the first element in the order.
	 * @param distanceFunction 				A distance function for consecutive elements in the order.
	 */
	explicit OnlineOrderOptimization(const std::function<double(const V &)> &firstDistanceFunction,
									 const std::function<double(const V &, const V &)> &distanceFunction,
									 std::function<void(const std::vector<V> &)> newBestOrderCallback)
			: distance_function(distanceFunction), first_distance_function(firstDistanceFunction),
			  new_best_order_callback(newBestOrderCallback) {
	}

	/**
	 * Run an iteration of the optimization algorithm; a single iteration should complete fairly quickly (< 1ms if possible).
	 * Repeatedly calling this function should progress towards the optimal ordering.
	 */
	virtual void iterate() = 0;

	/**
	 * Insert a new element anywhere in the order; this will generally be done according to some simple heuristic,
	 * with the element eventually migrating to its (locally) optimal position as iterate() is called repeatedly.
	 *
	 * @param item 		The item to insert.
	 */
	virtual void insert(V item) = 0;

	/**
	 * Delete an item from the ordering, declaring that it no longer needs to be visited.
	 *
	 * @param item 		The item to delete.
	 */
	virtual void remove(V item) = 0;

};


#endif //NEW_PLANNERS_ONLINEORDEROPTIMIZATION_H
