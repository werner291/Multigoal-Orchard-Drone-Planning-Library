// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by Werner Kroneman on 04/08/2023.
//

#ifndef MGODPL_PERIODICFULLREORDERINGTSPMETHODS_H
#define MGODPL_PERIODICFULLREORDERINGTSPMETHODS_H


#include <memory>
#include "IncrementalTSPMethods.h"

namespace mgodpl {
	namespace tsp_utils {

		UpdateTSPMethods make_periodic_full_reordering_tsp_methods(UpdateTSPMethods quickMethod,
																   UpdateTSPMethods slowMethod,
																   const double periodScalingFactor) {

			std::shared_ptr<size_t> changes_since_last_reordering = std::make_shared<size_t>(0);

			return UpdateTSPMethods {



			};

		}

		/**
 * A class that implements the IncrementalTSPMethods interface; it computes the ordering by: Initially using an expensive ordering method,
 * and then as goals are added or removed, it uses a quick method to update the ordering.
 *
 * When enough goals have been added, determined as a factor of the total number of goals in the current ordering,
 * the ordering is recomputed using the expensive method instead of the quick method.
 *
 * Note: While we speak of "expensive" and "quick" methods, the class really just wraps two methods,
 * and it is up to the user to decide which ones to put in each slot.
 *
 */
		class PeriodicFullReorderingTSPMethods : public IncrementalTSPMethods {

		public:
			/**
			 * Constructor.
			 * @param quickMethod 				The quick method.
			 * @param slowMethod 				The slow method.
			 * @param periodScalingFactor 		The factor of the total number of goals in the current ordering that determines when to use the slow method.
			 */
			PeriodicFullReorderingTSPMethods(const std::shared_ptr<IncrementalTSPMethods> &quickMethod,
											 const std::shared_ptr<IncrementalTSPMethods> &slowMethod,
											 const double periodScalingFactor);

			std::vector<size_t> initial_ordering(size_t n,
												 std::function<double(size_t, size_t)> distance,
												 std::function<double(size_t)> first_distance) override;

			std::vector<NewOrderingEntry> update_ordering_with_insertion(size_t old_n,
																		 std::function<double(const NewOrderingEntry &,
																							  const NewOrderingEntry &)> distance,
																		 std::function<double(const NewOrderingEntry &)> first_distance) override;

			std::vector<size_t> update_ordering_with_removal(size_t old_n,
															 size_t removed,
															 std::function<double(const size_t &, const size_t &)> distance,
			std::function<double(const size_t &)> first_distance) override;

		private:

			/// The quick method.
			const std::shared_ptr<IncrementalTSPMethods> quick_method;

			/// THe slow, expensive method.
			const std::shared_ptr<IncrementalTSPMethods> slow_method;

			/// A scaling parameter for the period.
			const double period_scaling_factor;

			/// The number of changes processed with the quick method since the last reordering with the slow method.
			size_t changes_since_last_reordering;

		};
	}
}




#endif //MGODPL_PERIODICFULLREORDERINGTSPMETHODS_H
