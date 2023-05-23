
#pragma once

#include "IncrementalTSPMethods.h"

/**
 * A class implementing simple incremental TSP methods
 * derived from the IncrementalTSPMethods base class.
 */
class SimpleIncrementalTSPMethods : public IncrementalTSPMethods {

public:
	/**
	 * Enumeration representing various strategies.
	 */
	enum Strategy {
		LastInFirstOut,
		FirstInFirstOut,
		FirstInSecondOut,
		LeastCostlyInsertion,
		Random
	};

private:
	Strategy strategy;  // The chosen strategy

public:
	/**
	 * Constructor for SimpleIncrementalTSPMethods.
	 * @param strategy: The chosen strategy.
	 */
	explicit SimpleIncrementalTSPMethods(Strategy strategy);

	/**
	 * Computes the initial ordering.
	 * @param n: The number of elements.
	 * @param distance: A function returning the distance between two elements.
	 * @param first_distance: A function returning the distance from the implicit starting point to an element.
	 * @return A vector of the initial ordering.
	 */
	std::vector<size_t> initial_ordering(size_t n,
										 std::function<double(size_t, size_t)> distance,
										 std::function<double(size_t)> first_distance) const override;

	/**
	 * Updates the ordering after a new element is inserted.
	 * @param old_n: The old number of elements.
	 * @param distance: A function returning the distance between two NewOrderingEntry elements.
	 * @param first_distance: A function returning the distance from the implicit starting point to a NewOrderingEntry element.
	 * @return A vector of the updated ordering with the new element.
	 */
	std::vector<NewOrderingEntry> update_ordering_with_insertion(size_t old_n,
																 std::function<double(const NewOrderingEntry &,
																					  const NewOrderingEntry &)> distance,
																 std::function<double(const NewOrderingEntry &)> first_distance) const override;

	/**
	 * Updates the ordering after an element is removed.
	 * @param old_n: The old number of elements.
	 * @param removed: The index of the removed element.
	 * @param distance: A function returning the distance between two size_t elements.
	 * @param first_distance: A function returning the distance from the implicit starting point to a size_t element.
	 * @return A vector of the updated ordering without the removed element.
	 */
	std::vector<size_t> update_ordering_with_removal(size_t old_n,
													 size_t removed,
													 std::function<double(const size_t &, const size_t &)> distance,
													 std::function<double(const size_t &)> first_distance) const override;

	size_t insertionPointByStrategy(size_t old_n,
									const std::function<double(const NewOrderingEntry &,
															   const NewOrderingEntry &)> &distance,
									const std::function<double(const NewOrderingEntry &)> &first_distance) const;
};
