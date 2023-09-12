// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 29-8-23.
//

#ifndef MGODPL_OMPLSHELLSPACE_H
#define MGODPL_OMPLSHELLSPACE_H

#include "../ShellConfigurationSpace.h"

#include "../../shell_space/OmplShellSpace.h"

namespace mgodpl::shell_configuration_space {

	template<typename ShellPoint>
	struct internal_point_t<OmplShellSpace<ShellPoint>>  {
		using type = ShellPoint;
	};

};

namespace mgodpl::approach_planning {

	/**
	 * @brief Find a pair of closest configurations between a shell and a goal region.
	 */
	template<typename ShellPoint>
	ShellPoint find_closest_shellpoint(
			const OmplShellSpace<ShellPoint>& shell_configuration_space,
			const ompl::base::Goal& goal_region) {
		return shell_configuration_space.pointNearGoal(&goal_region);
	}
}

#endif //MGODPL_OMPLSHELLSPACE_H
