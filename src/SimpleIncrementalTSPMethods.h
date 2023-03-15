// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-3-23.
//

#ifndef NEW_PLANNERS_SIMPLEINCREMENTALTSPMETHODS_H
#define NEW_PLANNERS_SIMPLEINCREMENTALTSPMETHODS_H

#include "IncrementalTSPMethods.h"

class SimpleIncrementalTSPMethods : public IncrementalTSPMethods {

public:
	enum Strategy {

		LastInFirstOut, FirstInFirstOut, FirstInSecondOut

	};

private:
	Strategy strategy;

public:
	explicit SimpleIncrementalTSPMethods(Strategy strategy);

	std::vector<size_t> initial_ordering(size_t n,
										 std::function<double(size_t, size_t)> distance,
										 std::function<double(size_t)> first_distance) const override;

	std::vector<NewOrderingEntry> update_ordering_with_insertion(size_t old_n,
																 std::function<double(const NewOrderingEntry &,
																					  const NewOrderingEntry &)> distance,
																 std::function<double(const NewOrderingEntry &)> first_distance) const override;

	std::vector<size_t> update_ordering_with_removal(size_t old_n,
													 size_t removed,
													 std::function<double(const size_t &, const size_t &)> distance,
													 std::function<double(const size_t &)> first_distance) const override;
};


#endif //NEW_PLANNERS_SIMPLEINCREMENTALTSPMETHODS_H
