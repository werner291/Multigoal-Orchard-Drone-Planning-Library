// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#ifndef NEW_PLANNERS_STARALPHACHULLSHELL_H
#define NEW_PLANNERS_STARALPHACHULLSHELL_H


#include <memory>
#include "../planning_scene_diff_message.h"
#include "WorkspaceShell.h"

struct StarAlphaShellPoint {
	CGAL::Delaunay_triangulation_3<CGAL::Epick>::Cell_handle cell;
	std::array<double, 4> barycentric_coordinates{};
};

class StarAlphaShell : public WorkspaceShell<StarAlphaShellPoint> {

	using K = CGAL::Epick;
	using Delaunay = CGAL::Delaunay_triangulation_3<K>;
	using Point = Delaunay::Point;

	Delaunay dt;
	Eigen::Vector3d global_barycenter;

	std::vector<StarAlphaShellPoint> find_exit_path(const StarAlphaShellPoint &from);

public:
	[[nodiscard]] Eigen::Vector3d arm_vector(const StarAlphaShellPoint &p) const override;

	[[nodiscard]] StarAlphaShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d surface_point(const StarAlphaShellPoint &p) const override;

	[[nodiscard]] std::shared_ptr<ShellPath<StarAlphaShellPoint>>
	path_from_to(const StarAlphaShellPoint &from, const StarAlphaShellPoint &to) const override;

	[[nodiscard]] double path_length(const std::shared_ptr<ShellPath<StarAlphaShellPoint>> &path) const override;

	StarAlphaShell(const AppleTreePlanningScene &scene_info);

};

std::shared_ptr<StarAlphaShell> paddedStarAlphaShell(const AppleTreePlanningScene &scene_info, double padding);

#endif //NEW_PLANNERS_STARALPHACHULLSHELL_H
