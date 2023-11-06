// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/6/23.
//

#ifndef MGODPL_JOINTSPACEPATH_H
#define MGODPL_JOINTSPACEPATH_H

#include "JointSpacePoint.h"

namespace mgodpl::moveit_facade {
	struct JointSpacePath {
		std::vector<moveit_facade::JointSpacePoint> path;
	};
}

#endif //MGODPL_JOINTSPACEPATH_H
