// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#ifndef NEW_PLANNERS_ORTOOLSTSPMETHODS_H
#define NEW_PLANNERS_ORTOOLSTSPMETHODS_H

#include "IncrementalTSPMethods.h"

class ORToolsTSPMethods : public IncrementalTSPMethods {

public:
	enum UpdateStrategy {

		LEAST_COSTLY_INSERT, FULL_REORDER

	};

private:
	UpdateStrategy update_strategy;

public:
	explicit ORToolsTSPMethods(UpdateStrategy updateStrategy);

	std::vector<size_t> initial_ordering(size_t n,
										 std::function<double(size_t, size_t)> distance,
										 std::function<double(size_t)> first_distance) override;

	std::vector<NewOrderingEntry> update_ordering(size_t old_n,
												  std::function<double(const NewOrderingEntry &,
																	   const NewOrderingEntry &)> distance,
												  std::function<double(const NewOrderingEntry &)> first_distance) override;


};

#endif //NEW_PLANNERS_ORTOOLSTSPMETHODS_H
