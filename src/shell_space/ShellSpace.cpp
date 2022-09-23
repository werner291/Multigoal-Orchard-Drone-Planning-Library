#include <moveit/robot_state/robot_state.h>
#include "../utilities/moveit.h"

struct Polar {
	double r;
	double azimuth;
	double altitude;
};

Polar pointToPolar(const Eigen::Vector3d &point) {
	Polar polar {};
	polar.r = point.norm();
	polar.azimuth = std::atan2(point.y(), point.x());
	polar.altitude = std::asin(point.z() / polar.r);
	return polar;
}

moveit::core::RobotState robotStateFromPointAndArmvec(const moveit::core::RobotModelConstPtr &drone,
													  const Eigen::Vector3d &desired_ee_pos,
													  const Eigen::Vector3d &armvec) {

	// Ensure armvec is normalized
	assert(std::abs(armvec.norm() - 1.0) < 1e-6);

	Polar polar = pointToPolar(armvec);

	Eigen::Quaterniond qd(Eigen::AngleAxisd(2.0 * M_PI - (M_PI / 2.0 - polar.azimuth), Eigen::Vector3d::UnitZ()));

	// Pick a base joint angle such that the arm is pointing in the right direction.

	moveit::core::RobotState st(drone);

	st.setVariablePositions({0.0, 0.0, 0.0,
									qd.x(), qd.y(), qd.z(), qd.w(),
									polar.altitude,
									0.0,
									0.0,
									0.0
							});

	st.update(true);

	// Apply a translation to the base to bring the end-effector to the desired position.

	setBaseTranslation(st, desired_ee_pos - st.getGlobalLinkTransform("end_effector").translation());

	st.update(true);

	// Sanity check: is the arm pointing in the right direction?
	{
		Eigen::Vector3d actual_armvec = st.getGlobalLinkTransform("arm2").translation() - st.getGlobalLinkTransform("arm").translation();
		actual_armvec.normalize();

		assert((actual_armvec - armvec).norm() < 1e-6);
	}

	return st;

}


