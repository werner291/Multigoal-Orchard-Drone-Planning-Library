// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_VISITATION_ORDER_H
#define MGODPL_VISITATION_ORDER_H

namespace mgodpl {

	/**
	 * Greedily compute a visitation order given a vector of distances from the initial point, and a matrix of distances between the target points.
	 *
	 * This function is a greedy method that always picks the closest target point that hasn't been visited yet.
	 *
	 * @param target_to_target_distances 	The matrix of distances between the target points.
	 * @param initial_state_distances 		The distances from the initial point to the target points.
	 * @return 								The visitation order, as target indices matching the order of the target_to_target_distances matrix.
	 */
	std::vector<size_t> visitation_order_greedy(const std::vector<std::vector<double>> &target_to_target_distances,
												const std::vector<double> &initial_state_distances);

}

#endif //MGODPL_VISITATION_ORDER_H
