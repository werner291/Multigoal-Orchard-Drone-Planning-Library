
#ifndef NEW_PLANNERS_ANYTIMEOPTIMALINSERTION_H
#define NEW_PLANNERS_ANYTIMEOPTIMALINSERTION_H


#include <vector>
#include <functional>
#include "OnlineOrderOptimization.h"

/**
 * An algorithm meant to progressively optimize the order of a set of elements,
 * minimizing the sum of the distances between them (and between the first and some starting point),
 * while making it possible to interrupt the optimization at any time and get a valid result.
 *
 * It should be possible to add new elements to the set, and change the distance function.
 *
 * TODO: Take some kind of smoothness parameter into account as well if possible!
 *
 * @tparam V 		The type of the elements.
 */
template<typename V>
class AnytimeOptimalInsertion : public OnlineOrderOptimization<V> {

	std::vector<V> visit_ordering;
public:
	const std::vector<V> &getVisitOrdering() const {
		return visit_ordering;
	}

public:

	/**
	 * Constructor. The firstDistanceFunction and distanceFunction may point to mutable data and need not be deterministic
	 * from this class' point of view; these should ideally represent some form of "latest knowledge" query.
	 *
	 * The class will dynamically update the ordering based on whatever the functions return.
	 *
	 * @param firstDistanceFunction 		Function that computes the distance between the first element and some starting point.
	 * @param distanceFunction 				Function that computes the distance between two elements.
	 * @param newBestOrderCallback 			Callback function to be called when a new best order is found.
	 */
	AnytimeOptimalInsertion(const std::function<double(const V &)> &firstDistanceFunction,
							const std::function<double(const V &, const V &)> &distanceFunction,
							const std::function<void(const std::vector<V> &)> &newBestOrderCallback) : OnlineOrderOptimization<V>(firstDistanceFunction,
																						   distanceFunction,
																						   newBestOrderCallback) {
	}

	/**
	 * Run an iteration of the optimization algorithm; a single iteration should complete fairly quickly (< 1ms if possible).
	 * Repeatedly calling this function should progress towards the optimal ordering.
	 */
	void iterate() override {

		// Pick a random index.
		size_t index = rand() % visit_ordering.size();

		// Remove the element.
		V removed = visit_ordering[index];
		visit_ordering.erase(visit_ordering.begin() + index);

		// Re-insert it.
		insert(removed);

		// Declare victory.
		OnlineOrderOptimization<V>::new_best_order_callback(visit_ordering);

	}

	/**
	 * Insert a new element anywhere in the order; this will generally be done according to some simple heuristic,
	 * with the element eventually migrating to its (locally) optimal position as iterate() is called repeatedly.
	 *
	 * @param item 		The item to insert.
	 */
	void insert(V item) override {

		if (visit_ordering.empty()) {
			// If the list is empty, just add the item.
			visit_ordering.push_back(item);
			return;
		} else {

			// Find the best place to insert the item.
			size_t best_insertion_spot = 0;

			// Best insertion spot is the spot that minimizes the sum of the distances before and after (if any) the insertion.
			double least_costly_distance = this->first_distance_function(item) + this->distance_function(item, visit_ordering[0]);

			for (size_t i = 1; i < visit_ordering.size() + 1; ++i) {

				// Distance before insertion.
				double distance = this->distance_function(visit_ordering[i-1], item);

				// Distance after insertion, if not at the end.
				if (i < visit_ordering.size()) {
					distance += this->distance_function(item, visit_ordering[i]);
				}

				// If this is the best spot so far, remember it.
				if (distance < least_costly_distance) {
					least_costly_distance = distance;
					best_insertion_spot = i;
				}
			}

			assert(best_insertion_spot <= visit_ordering.size());

			// Insert the item, at the end of the list is possible.
			visit_ordering.insert(visit_ordering.begin() + best_insertion_spot, item);

		}

	}

	/**
	 * Delete an item from the ordering, declaring that it no longer needs to be visited.
	 *
	 * Note: takes O(n) time in the number of items.
	 *
	 * @param item 		The item to delete.
	 */
	void remove(V item) override {

		auto pos = std::find(visit_ordering.begin(), visit_ordering.end(), item);

		assert(pos != visit_ordering.end());

		visit_ordering.erase(pos);
	}

};


#endif //NEW_PLANNERS_ANYTIMEOPTIMALINSERTION_H
