// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_MINIMUMENCLOSINGSPHERESHELL_H
#define MGODPL_MINIMUMENCLOSINGSPHERESHELL_H

#include <Eigen/Core>

#include "../Shell.h"
#include "../distance_matrix.h"

namespace mgodpl {

	struct GreatCirclePath {
		Eigen::Vector3d axis; ///< Vector perpendicular to the arc of rotation
		double angle; ///< Total rotation angle
		Eigen::Vector3d start_vec; ///< Start point relative to the center
	};

	struct SphereShell {
		Eigen::Vector3d center; ///< Center of the sphere
		double radius; ///< Radius of the sphere

		struct SpherePoint {
			Eigen::Vector3d point; ///< Point on the sphere's shell, relative to the center
		};

		using ShellPoint = SpherePoint;
		using ShellPath = GreatCirclePath;
	};

	// Implement all of Shell.h for SphereShell.

	template<>
	struct shell_point_t<SphereShell> {
		using type = SphereShell::SpherePoint;
	};

	template<>
	struct shell_path_t<SphereShell> {
		using type = GreatCirclePath;
	};

	namespace distance_matrix {
		/**
		 * @brief Computes the angular distance between two points on the shell of a sphere.
		 *
		 * @param shell The sphere shell.
		 * @param a First point on the shell.
		 * @param b Second point on the shell.
		 *
		 * @return The angular distance between points a and b on the shell.
		 */
		template<>
		double point_distance(const SphereShell &shell, const SphereShell::SpherePoint &a, const SphereShell::SpherePoint &b) {
			return std::acos(std::clamp(a.point.dot(b.point) / (shell.radius * shell.radius), -1.0, 1.0));
		}
	}

	/**
	 * @brief Converts a shell point to its Euclidean coordinates.
	 *
	 * @param shell The sphere shell.
	 * @param point A point on the shell.
	 *
	 * @return The Euclidean coordinates of the shell point.
	 */
	template<>
	SphereShell::SpherePoint shell_point_to_euclidean(const SphereShell &shell, const SphereShell::SpherePoint &point) {
		return {(point.point - shell.center).normalized() * shell.radius};
	}

	/**
	 * @brief Computes the normal vector at a given shell point.
	 *
	 * @param shell The sphere shell.
	 * @param point A point on the shell.
	 *
	 * @return The normal vector at the given shell point.
	 */
	template<>
	Eigen::Vector3d normal_at_shell_point(const SphereShell &shell, const SphereShell::SpherePoint &point) {
		return (point.point - shell.center).normalized();
	}

	/**
	 * @brief Projects a point in Euclidean space onto the shell.
	 *
	 * @param shell The sphere shell.
	 * @param point A point in Euclidean space.
	 *
	 * @return The projection of the point onto the shell.
	 */
	template<>
	SphereShell::SpherePoint project_euclidean_to_shell(const SphereShell &shell, const Eigen::Vector3d &point) {
		return {(point - shell.center).normalized() * shell.radius};
	}

}

#endif //MGODPL_MINIMUMENCLOSINGSPHERESHELL_H