//
// Created by werner on 28-11-22.
//

#ifndef NEW_PLANNERS_LOCALOPTIMIZER_H
#define NEW_PLANNERS_LOCALOPTIMIZER_H

const double PREFERRED_MINIMUM_CLEARANCE = 1.0;

#include "../utilities/moveit.h"
#include "../DirectPointCloudCollisionDetection.h"
#include "SegmentedRobotPath.h"

void localOptimizeSegmentedPath(const RobotPastTrace& robot_past,
								SegmentedRobotPath& lastPath, const
								DirectPointCloudCollisionDetection& collision_detector);


#endif //NEW_PLANNERS_LOCALOPTIMIZER_H
