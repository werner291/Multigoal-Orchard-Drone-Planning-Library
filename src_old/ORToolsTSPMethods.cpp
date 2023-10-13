// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ORToolsTSPMethods.h"
#include "utilities/traveling_salesman.h"

std::vector<size_t> mgodpl::tsp_utils::determine_tsp_order_ortools(size_t n ,std::function<double(size_t, size_t)> distance ,std::function<double(size_t)> first_distance) {

	return tsp_open_end(first_distance, distance, n);

}

mgodpl::tsp_utils::IncrementalTSPMethods mgodpl::tsp_utils::incremental_tsp_order_ortools_always_reorder() {
	return incrementalTspFromSimpleOrderngTSP(determine_tsp_order_ortools);
}
