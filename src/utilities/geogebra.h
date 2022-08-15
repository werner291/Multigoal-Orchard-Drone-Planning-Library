

#ifndef NEW_PLANNERS_GEOGEBRA_H
#define NEW_PLANNERS_GEOGEBRA_H

#include <Eigen/Core>
#include "../collision_free_shell/CuttingPlaneConvexHullShell.h"

void geogebra_dump_named_point(const Eigen::Vector3d &middle_proj_euc, const std::string& name);

void geogebra_dump_walk(const std::vector<ConvexHullPoint> &walk);

void geogebra_dump_named_walk(const std::vector<ConvexHullPoint> &walk, const std::string& name);

void geogebra_dump_named_face( const std::string& name, const Eigen::Vector3d& va, const Eigen::Vector3d& vb, const Eigen::Vector3d& vc);

void geogebra_dump_mesh(const shape_msgs::msg::Mesh &mesh);

#endif //NEW_PLANNERS_GEOGEBRA_H
