//
// Created by werner on 20-9-22.
//

#ifndef NEW_PLANNERS_WORKSPACESHELL_H
#define NEW_PLANNERS_WORKSPACESHELL_H

#include <Eigen/Core>
#include <moveit/robot_state/robot_state.h>

/**
 * An abstract path across a workspace shell.
 *
 * @tparam ShellPoint 		Type of points on the shell.
 */
template<typename ShellPoint>
class ShellPath {
	// Marker class, really only used as a common base class.

public:
	virtual double length() = 0;
};

/**
 * A path across a workspace shell, where the path is defined by a list of ShellPoints.
 * @tparam ShellPoint		Type of points on the shell.
 */
template<typename ShellPoint>
class PiecewiseLinearPath : public ShellPath<ShellPoint> {

public:
	std::vector<ShellPoint> points;

};

/**
 * A path across a workspace shell, where the shell is defined by a parametric curve.
 * @tparam ShellPoint		Type of points on the shell.
 */
template<typename ShellPoint>
class CurvePath : public ShellPath<ShellPoint> {

public:
	virtual ShellPoint at(double t) const = 0;

};

/**
 *
 * Abstract base class for a workspace shell, to later be used to define a Shell Configuration Space.
 *
 * This class depends only on Eigen, and is free from references to either OMPL or MoveIt.
 *
 * The ShellPoint type is abstract, as different definitions of the shell space may require different types.
 *
 * @tparam ShellPoint	The type of a point on the shell.
 *
 */
template<typename ShellPoint>
class WorkspaceShell {

public:

	/**
	 * Given a point on the shell, return an arm pointing vector.
	 *
	 * @param p 		The point on the shell.
	 * @return 			The arm pointing vector.
	 */
	virtual Eigen::Vector3d arm_vector(const ShellPoint& p) const = 0;

	/**
	 * Given a point in R^3, return the closest point on the shell.
	 *
	 * @param p 		The point in R^3.
	 * @return 			The closest point on the shell.
	 */
	virtual ShellPoint nearest_point_on_shell(const Eigen::Vector3d& p) const = 0;

	/**
	 * Given a point on the shell, return the associated point in R^3.
	 *
	 * @param p 			The point on the shell.
	 * @return 				The associated point in R^3.
	 */
	virtual Eigen::Vector3d surface_point(const ShellPoint& p) const = 0;

	/**
	 * Return a path from the given start point to the given end point over the surface of the shell.
	 *
	 * @param from 			The start point.
	 * @param to 			The end point.
	 * @return 				A path from the start point to the end point.
	 */
	virtual std::shared_ptr<ShellPath<ShellPoint>> path_from_to(const ShellPoint& from, const ShellPoint& to) const = 0;

	/**
	 * Return a random shell point on the shell near a given one.
	 */
	virtual ShellPoint random_near(const ShellPoint& p, double radius) const = 0;
};



#endif //NEW_PLANNERS_WORKSPACESHELL_H
