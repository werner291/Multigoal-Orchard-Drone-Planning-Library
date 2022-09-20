#include <moveit/robot_state/robot_state.h>

moveit::core::RobotState robotStateFromPointAndArmvec(const moveit::core::RobotModelConstPtr &drone,
													  const Eigen::Vector3d &desired_ee_pos,
													  const Eigen::Vector3d &armvec) {

	// Ensure armvec is normalized
	assert(std::abs(armvec.norm() - 1.0) < 1e-6);

	// Construct a quaternion rotation that puts the front of the drone facing the arm vector.
	Eigen::Quaterniond qd(Eigen::AngleAxisd(acos(armvec.z()), Eigen::Vector3d::UnitZ()));

	// Pick a base joint angle such that the arm is pointing in the right direction.
	double theta0 = atan2(armvec.topRows<2>().norm(), armvec.z());

	moveit::core::RobotState st(drone);

	st.setVariablePositions({
									0.0, 0.0, 0.0,
									qd.x(), qd.y(), qd.z(), qd.w(),
									theta0,
									0.0,
									0.0,
									0.0
							});

	st.update(true);

	// Apply a translation to the base to bring the end-effector to the desired position.
	Eigen::Vector3d offset = desired_ee_pos - st.getGlobalLinkTransform("end_effector").translation();

	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());

	st.update(true);

	// Sanity check: is the arm pointing in the right direction?
	{
		Eigen::Vector3d actual_armvec = st.getGlobalLinkTransform("link1").translation() - st.getGlobalLinkTransform("end_effector").translation();
		actual_armvec.normalize();
		assert((actual_armvec - armvec).norm() < 1e-6);
	}

	return st;

}