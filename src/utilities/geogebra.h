

#ifndef NEW_PLANNERS_GEOGEBRA_H
#define NEW_PLANNERS_GEOGEBRA_H

#include <Eigen/Core>
#include "../collision_free_shell/ConvexHullShell.h"

void geogebra_dump_named_point(const Eigen::Vector3d &middle_proj_euc, const std::string& name);

void geogebra_dump_walk(const std::vector<ConvexHullPoint> &walk);

#endif //NEW_PLANNERS_GEOGEBRA_H
