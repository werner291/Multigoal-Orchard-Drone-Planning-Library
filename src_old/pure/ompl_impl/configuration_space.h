// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 29-8-23.
//

#ifndef MGODPL_CONFIGURATION_SPACE_H
#define MGODPL_CONFIGURATION_SPACE_H

#include <ompl/base/Goal.h>

#include "../configuration_space.h"

namespace mgodpl::configuration_space {

	template<>
	struct configuration_t<ompl::base::Goal> {
		using type = ompl::base::State*;
	};

}

#endif //MGODPL_CONFIGURATION_SPACE_H
