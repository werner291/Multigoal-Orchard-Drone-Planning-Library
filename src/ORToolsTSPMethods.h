// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_ORTOOLSTSPMETHODS_H
#define NEW_PLANNERS_ORTOOLSTSPMETHODS_H

#include "IncrementalTSPMethods.h"

namespace mgodpl {
	namespace tsp_utils {

		std::vector<size_t> determine_tsp_order_ortools(size_t n ,std::function<double(size_t, size_t)> distance ,std::function<double(size_t)> first_distance);

		IncrementalTSPMethods incremental_tsp_order_ortools_always_reorder();

	}
}

#endif //NEW_PLANNERS_ORTOOLSTSPMETHODS_H
